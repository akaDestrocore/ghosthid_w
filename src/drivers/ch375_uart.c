/* CH375 Hardware Abstraction Callbacks for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "ch375/ch375.h"
#include "ch375/ch375_uart.h"

LOG_MODULE_REGISTER(ch375_hal, LOG_LEVEL_DBG);

/* Device context - stores UART device and GPIO spec */
typedef struct {
    const char *name;
    const struct device *uart_dev;
    struct gpio_dt_spec int_gpio;
} ch375_hw_context_t;

/* CH375 write command callback - uses native 9-bit UART */
static int ch375_write_cmd_cb(struct ch375_context *ctx, uint8_t cmd)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t data = CH375_CMD(cmd);  /* Bit 8 = 1 for command */
    
    /* Use Zephyr's native 9-bit UART function */
    uart_poll_out_u16(hw->uart_dev, data);
    
    LOG_DBG("%s: CMD 0x%04X", hw->name, data);
    return CH375_SUCCESS;
}

/* CH375 write data callback - uses native 9-bit UART */
static int ch375_write_data_cb(struct ch375_context *ctx, uint8_t data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val = CH375_DATA(data);  /* Bit 8 = 0 for data */
    
    /* Use Zephyr's native 9-bit UART function */
    uart_poll_out_u16(hw->uart_dev, val);
    
    LOG_DBG("%s: DATA 0x%04X", hw->name, val);
    return CH375_SUCCESS;
}

/* CH375 read data callback - uses native 9-bit UART */
static int ch375_read_data_cb(struct ch375_context *ctx, uint8_t *data)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    uint16_t val;
    int ret;
    
    /* Use Zephyr's native 9-bit UART function with timeout */
    int64_t start = k_uptime_get();
    int64_t timeout_ms = 500;
    
    while (1) {
        ret = uart_poll_in_u16(hw->uart_dev, &val);
        if (ret == 0) {
            /* Extract 8-bit data (ignore 9th bit on read) */
            *data = (uint8_t)(val & 0xFF);
            LOG_DBG("%s: READ 0x%04X -> 0x%02X", hw->name, val, *data);
            return CH375_SUCCESS;
        }
        
        /* Check timeout */
        if (k_uptime_get() - start >= timeout_ms) {
            LOG_ERR("%s: Read timeout", hw->name);
            return CH375_ERROR;
        }
        
        k_yield();
    }
}

/* CH375 query interrupt callback */
int ch375_query_int_cb(struct ch375_context *ctx)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    if (!device_is_ready(hw->int_gpio.port)) {
        return 0;
    }
    
    /* INT pin is active low */
    return gpio_pin_get_dt(&hw->int_gpio) == 0 ? 1 : 0;
}

/* Initialize CH375 hardware context and callbacks */
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
    
    /* Configure UART for 9-bit mode */
    ret = ch375_uart_configure_9bit(uart_dev, initial_baudrate);
    if (ret < 0) {
        LOG_ERR("%s: Failed to configure UART: %d", name, ret);
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

/* Change baudrate at runtime */
int ch375_hw_set_baudrate(struct ch375_context *ctx, uint32_t baudrate)
{
    ch375_hw_context_t *hw = (ch375_hw_context_t *)ch375_get_priv(ctx);
    
    return ch375_uart_configure_9bit(hw->uart_dev, baudrate);
}