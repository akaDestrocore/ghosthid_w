/* CH375 UART Implementation with STM32 9-bit Support */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

/* Direct STM32 HAL access for 9-bit mode */
#include <stm32f4xx_ll_usart.h>
#include <stm32f4xx_ll_bus.h>

#include "ch375/ch375.h"
#include "ch375/ch375_uart.h"

LOG_MODULE_REGISTER(ch375_uart, LOG_LEVEL_DBG);

/* Hardware context structure */
typedef struct {
    const char *name;
    const struct device *uart_dev;
    struct gpio_dt_spec int_gpio;
} ch375_hw_context_t;

/* Configure UART for 9-bit mode using LL drivers */
int ch375_uart_configure_9bit(const struct device *dev, uint32_t baudrate)
{
    USART_TypeDef *uart_instance;
    
    if (!device_is_ready(dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    /* Get USART instance from device */
    const char *dev_name = dev->name;
    
    if (strcmp(dev_name, "USART_2") == 0) {
        uart_instance = USART2;
    } else if (strcmp(dev_name, "USART_3") == 0) {
        uart_instance = USART3;
    } else if (strcmp(dev_name, "UART_4") == 0) {
        uart_instance = UART4;
    } else {
        LOG_ERR("Unknown UART device: %s", dev_name);
        return -ENOTSUP;
    }
    
    /* Disable UART before configuration */
    LL_USART_Disable(uart_instance);
    
    /* Configure for 9-bit mode */
    LL_USART_SetDataWidth(uart_instance, LL_USART_DATAWIDTH_9B);
    LL_USART_SetParity(uart_instance, LL_USART_PARITY_NONE);
    LL_USART_SetStopBitsLength(uart_instance, LL_USART_STOPBITS_1);
    
    /* Set baudrate */
    LL_USART_SetBaudRate(uart_instance, 
                         SystemCoreClock,
                         LL_USART_OVERSAMPLING_16,
                         baudrate);
    
    /* Enable TX and RX */
    LL_USART_SetTransferDirection(uart_instance, 
                                  LL_USART_DIRECTION_TX_RX);
    
    /* Re-enable UART */
    LL_USART_Enable(uart_instance);
    
    /* Wait for UART to be ready */
    k_busy_wait(100);
    
    LOG_INF("Configured %s for 9-bit mode at %d baud", dev_name, baudrate);
    return 0;
}

/* Write 16-bit data (9 data bits + 7 padding) */
int ch375_uart_write_u16_timeout(const struct device *dev, uint16_t data, 
                                  k_timeout_t timeout)
{
    USART_TypeDef *uart_instance;
    int64_t start_time = k_uptime_get();
    int64_t timeout_ms;
    
    if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
        timeout_ms = INT64_MAX;
    } else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
        timeout_ms = 0;
    } else {
        timeout_ms = timeout.ticks * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    }
    
    /* Get USART instance */
    const char *dev_name = dev->name;
    if (strcmp(dev_name, "USART_2") == 0) {
        uart_instance = USART2;
    } else if (strcmp(dev_name, "USART_3") == 0) {
        uart_instance = USART3;
    } else if (strcmp(dev_name, "UART_4") == 0) {
        uart_instance = UART4;
    } else {
        return -ENOTSUP;
    }
    
    /* Wait for TX buffer to be empty */
    while (!LL_USART_IsActiveFlag_TXE(uart_instance)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            return -ETIMEDOUT;
        }
        k_yield();
    }
    
    /* Write 9-bit data (only lower 9 bits used) */
    LL_USART_TransmitData9(uart_instance, data & 0x1FF);
    
    /* Wait for transmission complete */
    while (!LL_USART_IsActiveFlag_TC(uart_instance)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            return -ETIMEDOUT;
        }
        k_yield();
    }
    
    return 0;
}

/* Read 16-bit data with timeout */
int ch375_uart_read_u16_timeout(const struct device *dev, uint16_t *data, 
                                k_timeout_t timeout)
{
    USART_TypeDef *uart_instance;
    int64_t start_time = k_uptime_get();
    int64_t timeout_ms;
    
    if (!data) {
        return -EINVAL;
    }
    
    if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
        timeout_ms = INT64_MAX;
    } else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
        timeout_ms = 0;
    } else {
        timeout_ms = timeout.ticks * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    }
    
    /* Get USART instance */
    const char *dev_name = dev->name;
    if (strcmp(dev_name, "USART_2") == 0) {
        uart_instance = USART2;
    } else if (strcmp(dev_name, "USART_3") == 0) {
        uart_instance = USART3;
    } else if (strcmp(dev_name, "UART_4") == 0) {
        uart_instance = UART4;
    } else {
        return -ENOTSUP;
    }
    
    /* Wait for RX data */
    while (!LL_USART_IsActiveFlag_RXNE(uart_instance)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            return -ETIMEDOUT;
        }
        k_yield();
    }
    
    /* Read 9-bit data */
    *data = LL_USART_ReceiveData9(uart_instance);
    
    return 0;
}

/* ============================================================
 * CH375 CALLBACK IMPLEMENTATION - ADD BELOW HERE
 * ============================================================ */

/* Write command callback - uses 9-bit UART */
static int ch375_write_cmd_cb(struct ch375_context *ctx, uint8_t cmd)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t data = CH375_CMD(cmd);  /* Bit 8 = 1 for command */
    int ret;
    
    ret = ch375_uart_write_u16_timeout(hw->uart_dev, data, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: CMD write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    LOG_DBG("%s: CMD 0x%04X", hw->name, data);
    return CH375_SUCCESS;
}

/* Write data callback - uses 9-bit UART */
static int ch375_write_data_cb(struct ch375_context *ctx, uint8_t data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val = CH375_DATA(data);  /* Bit 8 = 0 for data */
    int ret;
    
    ret = ch375_uart_write_u16_timeout(hw->uart_dev, val, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: DATA write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    LOG_DBG("%s: DATA 0x%04X", hw->name, val);
    return CH375_SUCCESS;
}

/* Read data callback - uses 9-bit UART */
static int ch375_read_data_cb(struct ch375_context *ctx, uint8_t *data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val;
    int ret;
    
    ret = ch375_uart_read_u16_timeout(hw->uart_dev, &val, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: READ failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    /* Extract 8-bit data (ignore 9th bit on read) */
    *data = (uint8_t)(val & 0xFF);
    LOG_DBG("%s: READ 0x%04X -> 0x%02X", hw->name, val, *data);
    
    return CH375_SUCCESS;
}

/* Query interrupt callback */
static int ch375_query_int_cb(struct ch375_context *ctx)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    if (!device_is_ready(hw->int_gpio.port)) {
        return 0;
    }
    
    /* INT pin is active low */
    return gpio_pin_get_dt(&hw->int_gpio) == 0 ? 1 : 0;
}

/* Initialize CH375 hardware with proper 9-bit UART */
int ch375_hw_init(const char *name,
                  const struct device *uart_dev,
                  struct gpio_dt_spec int_gpio,
                  uint32_t initial_baudrate,
                  struct ch375_context **ctx_out)
{
    ch375_hw_context_t *hw;
    struct ch375_context *ctx;
    int ret;
    
    /* Allocate hardware context */
    hw = k_malloc(sizeof(ch375_hw_context_t));
    if (!hw) {
        LOG_ERR("Failed to allocate HW context");
        return -ENOMEM;
    }
    
    hw->name = name;
    hw->uart_dev = uart_dev;
    hw->int_gpio = int_gpio;
    
    /* Check UART device */
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("%s: UART device not ready", name);
        k_free(hw);
        return -ENODEV;
    }
    
    /* Configure UART for 9-bit mode - THIS IS THE CRITICAL FIX */
    ret = ch375_uart_configure_9bit(uart_dev, initial_baudrate);
    if (ret < 0) {
        LOG_ERR("%s: Failed to configure 9-bit UART: %d", name, ret);
        k_free(hw);
        return ret;
    }
    
    /* Configure INT GPIO */
    if (!device_is_ready(int_gpio.port)) {
        LOG_ERR("%s: INT GPIO not ready", name);
        k_free(hw);
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&int_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("%s: Failed to configure INT GPIO: %d", name, ret);
        k_free(hw);
        return ret;
    }
    
    /* Open CH375 context with callbacks */
    ret = ch375_open_context(&ctx,
                            ch375_write_cmd_cb,
                            ch375_write_data_cb,
                            ch375_read_data_cb,
                            ch375_query_int_cb,
                            hw);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("%s: Failed to open CH375 context: %d", name, ret);
        k_free(hw);
        return -EIO;
    }
    
    *ctx_out = ctx;
    LOG_INF("%s: Hardware initialized successfully", name);
    return 0;
}

/* Change baudrate and reconfigure 9-bit mode */
int ch375_hw_set_baudrate(struct ch375_context *ctx, uint32_t baudrate)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    LOG_INF("%s: Changing baudrate to %d", hw->name, baudrate);
    return ch375_uart_configure_9bit(hw->uart_dev, baudrate);
}