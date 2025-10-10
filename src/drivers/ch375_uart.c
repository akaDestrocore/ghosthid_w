/* CH375 UART Driver with 9-bit mode support for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ch375/ch375_uart.h"

LOG_MODULE_REGISTER(ch375_uart, LOG_LEVEL_DBG);

/* UART 9-bit mode implementation using Zephyr's native API */

int uart_poll_out_u16(const struct device *dev, uint16_t out_data)
{
    int ret;
    uint8_t data_low = out_data & 0xFF;
    uint8_t data_high = (out_data >> 8) & 0x01;  /* 9th bit */
    
    /* For STM32, we need to handle 9-bit mode specially */
    /* The 9th bit is typically in the TDR register's bit 8 */
    
    /* First, send the data with 9th bit information */
    /* This requires low-level register access for STM32 */
    
#ifdef CONFIG_SOC_FAMILY_STM32
    /* STM32-specific 9-bit handling */
    USART_TypeDef *uart_instance = (USART_TypeDef *)DEVICE_MMIO_GET(dev);
    
    /* Wait for TX empty */
    while (!(uart_instance->SR & USART_SR_TXE)) {
        k_yield();
    }
    
    /* Write 9-bit data to TDR */
    uart_instance->DR = out_data & 0x1FF;  /* 9 bits */
#else
    /* Generic fallback - send as two bytes */
    ret = uart_poll_out(dev, data_low);
    if (ret != 0) {
        return ret;
    }
    
    /* Send high bit as separate byte (not ideal but works) */
    if (data_high) {
        ret = uart_poll_out(dev, 0x01);
    }
#endif
    
    return 0;
}

int uart_poll_in_u16(const struct device *dev, uint16_t *in_data)
{
    int ret;
    uint16_t data = 0;
    
#ifdef CONFIG_SOC_FAMILY_STM32
    /* STM32-specific 9-bit handling */
    USART_TypeDef *uart_instance = (USART_TypeDef *)DEVICE_MMIO_GET(dev);
    
    /* Check if data is available */
    if (!(uart_instance->SR & USART_SR_RXNE)) {
        return -1;  /* No data available */
    }
    
    /* Read 9-bit data from RDR */
    data = uart_instance->DR & 0x1FF;  /* 9 bits */
    *in_data = data;
#else
    /* Generic fallback */
    uint8_t byte;
    ret = uart_poll_in(dev, &byte);
    if (ret != 0) {
        return ret;
    }
    *in_data = byte;
#endif
    
    return 0;
}

/* Asynchronous UART operations for better performance */
struct ch375_uart_data {
    uart_callback_t callback;
    void *user_data;
    struct k_sem tx_sem;
    struct k_sem rx_sem;
    uint16_t rx_buffer[64];
    uint16_t tx_buffer[64];
    size_t rx_len;
    size_t tx_len;
};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    struct ch375_uart_data *data = user_data;
    
    switch (evt->type) {
    case UART_TX_DONE:
        k_sem_give(&data->tx_sem);
        break;
        
    case UART_RX_RDY:
        data->rx_len = evt->data.rx.len;
        k_sem_give(&data->rx_sem);
        break;
        
    case UART_RX_DISABLED:
        LOG_DBG("RX disabled");
        break;
        
    default:
        break;
    }
}

int ch375_uart_init_async(const struct device *dev, struct ch375_uart_data *data)
{
    int ret;
    
    k_sem_init(&data->tx_sem, 0, 1);
    k_sem_init(&data->rx_sem, 0, 1);
    
    ret = uart_callback_set(dev, uart_cb, data);
    if (ret) {
        LOG_ERR("Failed to set UART callback: %d", ret);
        return ret;
    }
    
    /* Start RX */
    ret = uart_rx_enable(dev, (uint8_t *)data->rx_buffer, 
                        sizeof(data->rx_buffer), SYS_FOREVER_MS);
    if (ret) {
        LOG_ERR("Failed to enable RX: %d", ret);
        return ret;
    }
    
    return 0;
}

int ch375_uart_send_async(const struct device *dev, struct ch375_uart_data *data,
                          const uint16_t *buf, size_t len)
{
    int ret;
    
    if (len > ARRAY_SIZE(data->tx_buffer)) {
        return -EINVAL;
    }
    
    memcpy(data->tx_buffer, buf, len * sizeof(uint16_t));
    data->tx_len = len;
    
    ret = uart_tx(dev, (uint8_t *)data->tx_buffer, 
                 len * sizeof(uint16_t), SYS_FOREVER_MS);
    if (ret) {
        LOG_ERR("Failed to start TX: %d", ret);
        return ret;
    }
    
    /* Wait for TX complete */
    ret = k_sem_take(&data->tx_sem, K_MSEC(500));
    if (ret) {
        LOG_ERR("TX timeout");
        return ret;
    }
    
    return 0;
}

int ch375_uart_recv_async(const struct device *dev, struct ch375_uart_data *data,
                         uint16_t *buf, size_t *len, k_timeout_t timeout)
{
    int ret;
    
    /* Wait for RX data */
    ret = k_sem_take(&data->rx_sem, timeout);
    if (ret) {
        return ret;  /* Timeout */
    }
    
    if (data->rx_len > 0) {
        size_t copy_len = MIN(data->rx_len, *len);
        memcpy(buf, data->rx_buffer, copy_len * sizeof(uint16_t));
        *len = copy_len;
        
        /* Restart RX */
        uart_rx_enable(dev, (uint8_t *)data->rx_buffer,
                      sizeof(data->rx_buffer), SYS_FOREVER_MS);
        
        return 0;
    }
    
    return -ENODATA;
}