/* CH375 UART Implementation - PROPER TIMEOUT ERROR HANDLING */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <stm32f4xx_ll_usart.h>
#include <stm32f4xx_ll_bus.h>

#include "ch375/ch375.h"
#include "ch375/ch375_uart.h"

LOG_MODULE_REGISTER(ch375_uart, LOG_LEVEL_DBG);

typedef struct {
    const char *name;
    const struct device *uart_dev;
    USART_TypeDef *uart_instance;
    struct gpio_dt_spec int_gpio;
} ch375_hw_context_t;

static USART_TypeDef *get_uart_instance(const struct device *dev);

/* Configure UART for 9-bit mode */
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
    k_busy_wait(100);
    
    /* Clear flags */
    (void)uart_instance->SR;
    (void)uart_instance->DR;
    
    /* Configure CR1 */
    tmpreg = uart_instance->CR1;
    tmpreg &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | 
                USART_CR1_TE | USART_CR1_RE);
    tmpreg |= USART_CR1_M | USART_CR1_TE | USART_CR1_RE;
    uart_instance->CR1 = tmpreg;
    
    /* Configure CR2 - 1 stop bit */
    tmpreg = uart_instance->CR2;
    tmpreg &= ~USART_CR2_STOP;
    uart_instance->CR2 = tmpreg;
    
    /* Configure CR3 */
    uart_instance->CR3 = 0;
    
    /* Set baud rate */
    uint32_t apb_clock;
    if (uart_instance == USART2 || uart_instance == USART3 || uart_instance == UART4) {
        apb_clock = SystemCoreClock / 4;
    } else {
        apb_clock = SystemCoreClock / 2;
    }
    
    uint32_t usartdiv = (apb_clock + (baudrate / 2)) / baudrate;
    uart_instance->BRR = usartdiv;
    
    LOG_INF("APB clock: %u Hz, BRR value: 0x%04X", apb_clock, usartdiv);
    
    /* Enable UART */
    uart_instance->CR1 |= USART_CR1_UE;
    k_busy_wait(2000);
    
    /* Clear flags */
    (void)uart_instance->SR;
    (void)uart_instance->DR;

    k_msleep(5);
    ch375_uart_flush_rx(dev);
    
    LOG_INF("UART configuration complete");
    return 0;
}

void ch375_uart_flush_rx(const struct device *dev)
{
    USART_TypeDef *uart_instance;
    uint16_t dummy;
    int timeout = 100;
    
    uart_instance = get_uart_instance(dev);
    if (!uart_instance) {
        return;
    }
    
    while (timeout-- > 0) {
        if (uart_instance->SR & USART_SR_RXNE) {
            dummy = uart_instance->DR;
            (void)dummy;
        } else {
            break;
        }
        k_busy_wait(10);
    }
    
    /* Clear error flags */
    if (uart_instance->SR & USART_SR_ORE) {
        (void)uart_instance->SR;
        (void)uart_instance->DR;
    }
    if (uart_instance->SR & USART_SR_FE) {
        (void)uart_instance->DR;
    }
    if (uart_instance->SR & USART_SR_NE) {
        (void)uart_instance->DR;
    }
}

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
    
    /* Write data */
    uart_instance->DR = data & 0x01FF;
    
    /* Wait for TC */
    while (!(uart_instance->SR & USART_SR_TC)) {
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            LOG_ERR("TC timeout");
            return -ETIMEDOUT;
        }
        k_busy_wait(10);
    }
    
    k_busy_wait(100);
    return 0;
}

/* CRITICAL: Use SHORT timeout for data reads to detect short packets */
int ch375_uart_read_u16_timeout(const struct device *dev, uint16_t *data, 
                                k_timeout_t timeout)
{
    USART_TypeDef *uart_instance;
    int64_t start_time = k_uptime_get();
    int64_t timeout_ms;
    uint32_t attempts = 0;
    uint32_t sr_reg;
    
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
    while (1) {
        sr_reg = uart_instance->SR;
        
        if (sr_reg & USART_SR_RXNE) {
            break;
        }
        
        if ((k_uptime_get() - start_time) >= timeout_ms) {
            /* Return specific timeout error - this is EXPECTED for short packets */
            return -ETIMEDOUT;
        }
        
        attempts++;
        
        /* Clear error flags */
        if (sr_reg & USART_SR_ORE) {
            (void)uart_instance->SR;
            (void)uart_instance->DR;
            continue;
        }
        if (sr_reg & USART_SR_FE) {
            (void)uart_instance->DR;
            continue;
        }
        if (sr_reg & USART_SR_NE) {
            (void)uart_instance->DR;
            continue;
        }
        
        k_busy_wait(10);
    }
    
    /* Read data */
    *data = uart_instance->DR & 0x01FF;
    
    LOG_DBG("RX success after %u attempts, took %lld ms, data=0x%04X", 
           attempts, (k_uptime_get() - start_time), *data);
    
    return 0;
}

static USART_TypeDef *get_uart_instance(const struct device *dev)
{
    if (dev == DEVICE_DT_GET(DT_NODELABEL(usart2))) {
        return USART2;
    } else if (dev == DEVICE_DT_GET(DT_NODELABEL(usart3))) {
        return USART3;
    } else if (dev == DEVICE_DT_GET(DT_NODELABEL(uart4))) {
        return UART4;
    }
    
    return NULL;
}

/* CH375 CALLBACKS */

static int ch375_write_cmd_cb(struct ch375_context *ctx, uint8_t cmd)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t data = CH375_CMD(cmd);
    int ret;
    
    ret = ch375_uart_write_u16_timeout(hw->uart_dev, data, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: CMD write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    LOG_DBG("%s: CMD 0x%04X", hw->name, data);
    return CH375_SUCCESS;
}

static int ch375_write_data_cb(struct ch375_context *ctx, uint8_t data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val = CH375_DATA(data);
    int ret;

    ret = ch375_uart_write_u16_timeout(hw->uart_dev, val, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: DATA write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    LOG_DBG("%s: DATA 0x%04X", hw->name, val);
    return CH375_SUCCESS;
}

/* CRITICAL: Use SHORT timeout (50ms) when reading data bytes
 * This allows detection of short packets from CH375
 */
static int ch375_read_data_cb(struct ch375_context *ctx, uint8_t *data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val;
    int ret;

    /* SHORT timeout of 50ms for data reads */
    ret = ch375_uart_read_u16_timeout(hw->uart_dev, &val, K_MSEC(50));
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            /* Return CH375_TIMEOUT so read_block_data can detect short packets */
            LOG_DBG("%s: READ timeout (normal for short packets)", hw->name);
            return CH375_TIMEOUT;
        }
        LOG_ERR("%s: READ failed: %d", hw->name, ret);
        return CH375_ERROR;
    }
    
    *data = (uint8_t)(val & 0xFF);
    LOG_DBG("%s: READ 0x%04X -> 0x%02X", hw->name, val, *data);
    
    return CH375_SUCCESS;
}

static int ch375_query_int_cb(struct ch375_context *ctx)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    if (!device_is_ready(hw->int_gpio.port)) {
        return 0;
    }
    
    return gpio_pin_get_dt(&hw->int_gpio) == 0 ? 1 : 0;
}

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
    
    uart_instance = get_uart_instance(uart_dev);
    if (!uart_instance) {
        LOG_ERR("%s: Unknown UART device", name);
        return -ENOTSUP;
    }
    
    hw = k_malloc(sizeof(ch375_hw_context_t));
    if (!hw) {
        LOG_ERR("Failed to allocate HW context");
        return -ENOMEM;
    }
    
    hw->name = name;
    hw->uart_dev = uart_dev;
    hw->uart_instance = uart_instance;
    hw->int_gpio = int_gpio;
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("%s: UART device not ready", name);
        k_free(hw);
        return -ENODEV;
    }
    
    ret = ch375_uart_configure_9bit(uart_dev, initial_baudrate);
    if (ret < 0) {
        LOG_ERR("%s: Failed to configure 9-bit UART: %d", name, ret);
        k_free(hw);
        return ret;
    }
    
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

int ch375_hw_set_baudrate(struct ch375_context *ctx, uint32_t baudrate)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    LOG_INF("%s: Changing baudrate to %d", hw->name, baudrate);
    return ch375_uart_configure_9bit(hw->uart_dev, baudrate);
}