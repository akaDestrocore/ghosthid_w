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
    USART_TypeDef *uart_instance;  // Store instance pointer
    struct gpio_dt_spec int_gpio;
} ch375_hw_context_t;

/* Forward declaration */
static USART_TypeDef *get_uart_instance(const struct device *dev);

/* Configure UART for 9-bit mode using LL drivers */
int ch375_uart_configure_9bit(const struct device *dev, uint32_t baudrate)
{
    USART_TypeDef *uart_instance;
    uint32_t tmpreg;
    
    if (!device_is_ready(dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    uart_instance = get_uart_instance(dev);
    if (!uart_instance) {
        LOG_ERR("Unknown UART device: %s", dev->name);
        return -ENOTSUP;
    }
    
    LOG_INF("Configuring %s for 9-bit mode at %d baud", dev->name, baudrate);
    
    /* Disable UART */
    uart_instance->CR1 &= ~USART_CR1_UE;
    
    /* Wait for disable */
    k_busy_wait(100);
    
    /* Clear all flags */
    uart_instance->SR = 0;
    
    /* Configure CR1 register */
    tmpreg = uart_instance->CR1;
    
    /* Clear M, PCE, PS, TE and RE bits */
    tmpreg &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | 
                USART_CR1_TE | USART_CR1_RE);
    
    /* Set 9-bit mode (M=1), no parity (PCE=0), enable TX and RX */
    tmpreg |= USART_CR1_M | USART_CR1_TE | USART_CR1_RE;
    
    uart_instance->CR1 = tmpreg;
    
    /* Configure CR2 - 1 stop bit */
    tmpreg = uart_instance->CR2;
    tmpreg &= ~USART_CR2_STOP;
    uart_instance->CR2 = tmpreg;
    
    /* Configure CR3 - no hardware flow control */
    uart_instance->CR3 = 0;
    
    /* Set baud rate 
     * BRR = (UART_CLOCK + (baudrate/2)) / baudrate
     * For STM32F4, USART2/3 are on APB1 (typically 42MHz)
     * UART4 is also on APB1
     */
    uint32_t apb_clock;
    
    if (uart_instance == USART2 || uart_instance == USART3 || uart_instance == UART4) {
        /* APB1 clock */
        apb_clock = SystemCoreClock / 4;  // Assuming APB1 = HCLK/4 (42MHz)
    } else {
        /* APB2 clock */
        apb_clock = SystemCoreClock / 2;  // Assuming APB2 = HCLK/2 (84MHz)
    }
    
    uint32_t usartdiv = (apb_clock + (baudrate / 2)) / baudrate;
    uart_instance->BRR = usartdiv;
    
    LOG_INF("APB clock: %u Hz, BRR value: 0x%04X", apb_clock, usartdiv);
    
    /* Enable UART */
    uart_instance->CR1 |= USART_CR1_UE;
    
    /* Wait for UART to be ready */
    k_busy_wait(2000);  // 2ms
    
    /* Clear status flags again */
    uart_instance->SR = 0;
    
    /* Read DR to clear RXNE if set */
    (void)uart_instance->DR;
    
    LOG_INF("UART configuration complete");
    
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
    
    uart_instance = get_uart_instance(dev);
    if (!uart_instance) {
        return -ENOTSUP;
    }
    
    /* Wait for TX empty */
    while (!(uart_instance->SR & USART_SR_TXE)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            LOG_ERR("TX timeout");
            return -ETIMEDOUT;
        }
        k_busy_wait(10);
    }
    
    /* Write 9-bit data (only bits 0-8 are used) */
    uart_instance->DR = data & 0x01FF;
    
    /* Wait for transmission complete */
    while (!(uart_instance->SR & USART_SR_TC)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            LOG_ERR("TC timeout");
            return -ETIMEDOUT;
        }
        k_busy_wait(10);
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
    uint32_t attempts = 0;
    
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
    
    uart_instance = get_uart_instance(dev);
    if (!uart_instance) {
        return -ENOTSUP;
    }
    
    /* Wait for RX data */
    while (!(uart_instance->SR & USART_SR_RXNE)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            LOG_ERR("RX timeout after %lld ms (attempts=%u, SR=0x%08X)", 
                   (k_uptime_get() - start_time), attempts, uart_instance->SR);
            return -ETIMEDOUT;
        }
        
        attempts++;
        
        /* Check for errors */
        if (uart_instance->SR & USART_SR_ORE) {
            LOG_ERR("Overrun error");
            /* Clear by reading SR then DR */
            (void)uart_instance->SR;
            (void)uart_instance->DR;
        }
        if (uart_instance->SR & USART_SR_FE) {
            LOG_ERR("Framing error");
            (void)uart_instance->DR;  // Clear by reading DR
        }
        if (uart_instance->SR & USART_SR_NE) {
            LOG_ERR("Noise error");
            (void)uart_instance->DR;  // Clear by reading DR
        }
        
        k_busy_wait(10);
    }
    
    /* Read 9-bit data */
    *data = uart_instance->DR & 0x01FF;
    
    LOG_DBG("RX success after %u attempts, took %lld ms, data=0x%04X", 
           attempts, (k_uptime_get() - start_time), *data);
    
    return 0;
}

/* Get USART instance from device - use DT macros */
static USART_TypeDef *get_uart_instance(const struct device *dev)
{
    /* Check against known UART device tree nodes */
    if (dev == DEVICE_DT_GET(DT_NODELABEL(usart2))) {
        return USART2;
    } else if (dev == DEVICE_DT_GET(DT_NODELABEL(usart3))) {
        return USART3;
    } else if (dev == DEVICE_DT_GET(DT_NODELABEL(uart4))) {
        return UART4;
    }
    
    return NULL;
}

/* ============================================================
 * CH375 CALLBACK IMPLEMENTATION
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
    USART_TypeDef *uart_instance;
    int ret;
    
    /* Determine UART instance */
    uart_instance = get_uart_instance(uart_dev);
    if (!uart_instance) {
        LOG_ERR("%s: Unknown UART device", name);
        return -ENOTSUP;
    }
    
    /* Allocate hardware context */
    hw = k_malloc(sizeof(ch375_hw_context_t));
    if (!hw) {
        LOG_ERR("Failed to allocate HW context");
        return -ENOMEM;
    }
    
    hw->name = name;
    hw->uart_dev = uart_dev;
    hw->uart_instance = uart_instance;
    hw->int_gpio = int_gpio;
    
    /* Check UART device */
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("%s: UART device not ready", name);
        k_free(hw);
        return -ENODEV;
    }
    
    /* Configure UART for 9-bit mode */
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