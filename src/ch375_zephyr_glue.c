/*
 * ch375_zephyr_glue.c
 *
 * Glue: CH375 core <-> Zephyr parity UART + GPIO INT pins
 *
 * This file implements:
 *   - write_cmd, write_data, read_data, query_int callbacks for CH375 core
 *   - per-device container (DeviceInput_t) holding parity ctx and gpio info
 *
 * Assumptions (matching your overlay):
 *   * UART labels: "UART_2", "UART_3" (you used these labels in overlay)
 *   * INT pins: GPIOE pin numbers are passed when calling device_init()
 *
 * NOTE: the CH375 core expects write_cmd to send CMD (9th bit == 1),
 * write_data to send DATA (9th bit == 0). We use ch375_parity_send_cmd/data.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>

#include "ch375.h"
#include "ch375_interface.h"
#include "ch375_uart_parity.h"

LOG_MODULE_REGISTER(ch375_glue, LOG_LEVEL_DBG);

typedef struct DeviceInput_t {
    const char *name;
    struct ch375_parity_ctx parity;
    const struct device *gpio_dev;
    gpio_pin_t gpio_pin;
    int gpio_flags; /* usually GPIO_INPUT | GPIO_PULL_UP */
    CH375_Context_t *ch375_ctx; /* allocated by ch375_openContext */
} DeviceInput_t;

/* two devices: CH375A (GPIOE14) and CH375B (GPIOE15) by your request */
static DeviceInput_t s_devices[2];

/* forward declarations — these are the callbacks passed to ch375_openContext() */
static int zephyr_write_cmd(CH375_Context_t *context, uint8_t cmd);
static int zephyr_write_data(CH375_Context_t *context, uint8_t data);
static int zephyr_read_data(CH375_Context_t *context, uint8_t *data);
static int zephyr_query_int(CH375_Context_t *context);

/* Initialize one device's parity UART + INT GPIO
 * uart_label: e.g. "UART_2"
 * gpio_label: device name like "GPIOE" or use device_get_binding() on label
 * gpio_pin: pin number (e.g. 14)
 */
int ch375_zephyr_device_init(int idx, const char *name, const char *uart_label,
                             const char *gpio_label, gpio_pin_t gpio_pin, uint32_t initial_baud)
{
    if (idx < 0 || idx >= ARRAY_SIZE(s_devices)) return -EINVAL;
    DeviceInput_t *d = &s_devices[idx];
    memset(d, 0, sizeof(*d));
    d->name = name;

    /* open parity UART */
    int rc = ch375_parity_open(&d->parity, uart_label, initial_baud);
    if (rc) {
        LOG_ERR("parity open fail %s (%d)", uart_label, rc);
        return rc;
    }

    /* bind GPIO device for INT pin */
    d->gpio_dev = device_get_binding(gpio_label);
    if (!d->gpio_dev) {
        LOG_ERR("gpio device '%s' not found", gpio_label);
        ch375_parity_close(&d->parity);
        return -ENODEV;
    }
    d->gpio_pin = gpio_pin;
    d->gpio_flags = GPIO_INPUT | GPIO_PULL_UP;
    rc = gpio_pin_configure(d->gpio_dev, d->gpio_pin, d->gpio_flags);
    if (rc) {
        LOG_ERR("gpio_pin_configure failed %d", rc);
        ch375_parity_close(&d->parity);
        return rc;
    }

    /* open CH375 context: allocate and supply the four function pointers */
    rc = ch375_openContext(&d->ch375_ctx,
                           zephyr_write_cmd,
                           zephyr_write_data,
                           zephyr_read_data,
                           zephyr_query_int,
                           d); /* priv = pointer to DeviceInput_t */
    if (rc != 0) {
        LOG_ERR("ch375_openContext failed %d", rc);
        ch375_parity_close(&d->parity);
        return rc;
    }

    LOG_INF("ch375 device %s initialised (uart=%s gpio=%s:%d)", name, uart_label, gpio_label, gpio_pin);
    return 0;
}

/* helper to fetch DeviceInput_t from CH375_Context_t priv */
static DeviceInput_t *get_deviceinput_from_context(CH375_Context_t *ctx)
{
    if (!ctx) return NULL;
    return (DeviceInput_t *)ch375_getPriv(ctx);
}

/* callbacks used by ch375 core */
static int zephyr_write_cmd(CH375_Context_t *context, uint8_t cmd)
{
    DeviceInput_t *d = get_deviceinput_from_context(context);
    if (!d) return CH375_PARAM_INVALID;
    return ch375_parity_send_cmd(&d->parity, cmd);
}

static int zephyr_write_data(CH375_Context_t *context, uint8_t data)
{
    DeviceInput_t *d = get_deviceinput_from_context(context);
    if (!d) return CH375_PARAM_INVALID;
    return ch375_parity_send_data(&d->parity, data);
}

static int zephyr_read_data(CH375_Context_t *context, uint8_t *data)
{
    DeviceInput_t *d = get_deviceinput_from_context(context);
    if (!d || !data) return CH375_PARAM_INVALID;
    /* default timeout: 500ms as in original code */
    int rc = ch375_parity_read_byte_timeout(&d->parity, data, 500);
    if (rc == 0) return CH375_SUCCESS;
    if (rc == -ETIMEDOUT) return CH375_TIMEOUT;
    return CH375_ERROR;
}

static int zephyr_query_int(CH375_Context_t *context)
{
    DeviceInput_t *d = get_deviceinput_from_context(context);
    if (!d || !d->gpio_dev) return 0;
    int val = gpio_pin_get(d->gpio_dev, d->gpio_pin);
    /* original design: INT is active low (0 -> interrupt) */
    return (val == 0) ? 1 : 0;
}

/* helper: reconfigure device UART to a new baud (e.g. after ch375_hostInit) */
int ch375_zephyr_set_baud(int idx, uint32_t new_baud)
{
    if (idx < 0 || idx >= ARRAY_SIZE(s_devices)) return -EINVAL;
    DeviceInput_t *d = &s_devices[idx];
    if (!d->parity.uart_dev) return -ENODEV;
    struct uart_config cfg = {
        .baudrate = new_baud,
        .data_bits = UART_CFG_DATA_BITS_8,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .parity = UART_CFG_PARITY_NONE,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    int rc = uart_configure(d->parity.uart_dev, &cfg);
    if (rc) {
        LOG_ERR("uart_configure to %u failed %d", new_baud, rc);
        return rc;
    }
    d->parity.baseline_cfg = cfg;
    d->parity.baud = new_baud;
    d->parity.char_time_ms = _compute_char_time_ms(new_baud);
    return 0;
}

/* provide accessors for main/forwarder */
CH375_Context_t *ch375_zephyr_get_context(int idx)
{
    if (idx < 0 || idx >= ARRAY_SIZE(s_devices)) return NULL;
    return s_devices[idx].ch375_ctx;
}
struct ch375_parity_ctx *ch375_zephyr_get_parity(int idx)
{
    if (idx < 0 || idx >= ARRAY_SIZE(s_devices)) return NULL;
    return &s_devices[idx].parity;
}
