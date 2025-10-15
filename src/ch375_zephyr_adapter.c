/*
 * ch375_zephyr_adapter.c (static-pool variant)
 *
 * Zephyr adapter implementing ch375_interface I/O (write_cmd, write_data, read_data, query_int)
 * Uses Zephyr UART polling API and GPIO callback + k_sem for INT signaling.
 *
 * This variant avoids dynamic allocation (k_malloc) by keeping a small static pool
 * of adapter instances.
 */

#include "ch375_zephyr_adapter.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h> /* for CONTAINER_OF */
#include <string.h>

LOG_MODULE_REGISTER(ch375_zephyr_adapter, LOG_LEVEL_INF);

/* CH375 API / error codes */
#include "ch375.h"
#include "ch375_interface.h"

/* Config: maximum number of CH375 instances supported by this adapter pool */
#ifndef MAX_CH375_INSTANCES
#define MAX_CH375_INSTANCES 2
#endif

/* --------- Forward declarations for STM32 register helpers (if present) --------- */
/* These are declared before they are used in ch375_zephyr_open to avoid implicit
 * function declarations and type conflicts during compilation.
 */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
/* Map a label string to USART_TypeDef* (returns NULL if label unknown) */
static USART_TypeDef *get_usart_from_label(const char *label);

/* Configure USART peripheral for 9-bit data (M = 1) and enable TE/RE */
static void configure_usart_for_9bit(USART_TypeDef *usart);

/* Write a 9-bit value (0..0x1FF). Returns 0 on success, negative on error */
static int usart_write9(USART_TypeDef *usart, uint16_t value);

/* Read a 9-bit value into out within timeout_ms ms. Returns 0 on success */
static int usart_read9(USART_TypeDef *usart, uint16_t *out, uint32_t timeout_ms);
#endif /* CONFIG_SOC_SERIES_STM32F4X */

/* --------- End forward declarations --------- */

/* priv structure passed to ch375_openContext() */
typedef struct {
    bool used;
    const struct device *uart;
    const struct device *gpio;
    gpio_pin_t int_pin;
    struct gpio_callback gcb;
    struct k_sem int_sem;
    uint32_t rx_timeout_ms;
    /* optional: store labels for logging convenience */
    const char *uart_label;
    const char *gpio_label;
} ch375_adapter_priv_t;

/* static pool */
static ch375_adapter_priv_t s_pool[MAX_CH375_INSTANCES];

/* forward declarations for functions passed to ch375_openContext() */
static int z_write_cmd(CH375_Context_t *context, uint8_t cmd);
static int z_write_data(CH375_Context_t *context, uint8_t data);
static int z_read_data(CH375_Context_t *context, uint8_t *data);
static int z_query_int(CH375_Context_t *context);
static void ch375_gpio_callback(const struct device *gpiodev, struct gpio_callback *cb, gpio_port_pins_t pins);

/* helper: allocate a free slot from the static pool */
static ch375_adapter_priv_t *alloc_slot(const char *uart_label, const char *gpio_label)
{
    for (int i = 0; i < MAX_CH375_INSTANCES; ++i) {
        if (!s_pool[i].used) {
            /* zero and mark used */
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].used = true;
            s_pool[i].uart_label = uart_label;
            s_pool[i].gpio_label = gpio_label;
            return &s_pool[i];
        }
    }
    return NULL;
}

/* helper: free slot */
static void free_slot(ch375_adapter_priv_t *p)
{
    if (!p) return;
    /* clear callback if registered (safe to call even if not) */
    if (p->gpio) {
        gpio_remove_callback(p->gpio, &p->gcb);
    }
    p->used = false;
    memset(p, 0, sizeof(*p));
}

/**
 * @brief Open CH375 on a Zephyr UART + INT GPIO
 */
int ch375_zephyr_open(const char *uart_label,
                      const char *gpio_label,
                      uint32_t int_pin,
                      CH375_Context_t **ctx_out)
{
    int ret;
    ch375_adapter_priv_t *priv = NULL;
    const struct device *uart_dev;
    const struct device *gpio_dev;

    if (!uart_label || !gpio_label || !ctx_out) {
        LOG_ERR("invalid params");
        return CH375_PARAM_INVALID;
    }

    uart_dev = device_get_binding(uart_label);
    if (!uart_dev) {
        LOG_ERR("UART device '%s' not found", uart_label);
        return CH375_PARAM_INVALID;
    }
    LOG_INF("UART binding ok: %s -> %p", uart_label, uart_dev);

    /* === TAKE REGISTER CONTROL (disable Zephyr UART driver IRQs, then set 9-bit mode) === */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    {
        /* Try to map label/name to the MCU USART instance (USART2/3) */
        const char *try_name = uart_label ? uart_label : (uart_dev ? uart_dev->name : NULL);
        USART_TypeDef *usart_for_label = get_usart_from_label(try_name);

        if (usart_for_label) {
            /* Disable Zephyr driver's IRQ activity for this UART device to avoid races.
             * We don't attempt to call UART callback APIs that may differ between Zephyr versions.
             */
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
            uart_irq_rx_disable(uart_dev);
            uart_irq_tx_disable(uart_dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

            /* Now put hardware into 9-bit mode and enable TE/RE */
            configure_usart_for_9bit(usart_for_label);

            LOG_INF("Took register control of %s for 9-bit mode", try_name ? try_name : "(unknown)");
        } else {
            LOG_DBG("No direct USART register mapping for '%s' (will use Zephyr poll IO fallback)", try_name ? try_name : "(null)");
        }
    }
#endif /* CONFIG_SOC_SERIES_STM32F4X */

    gpio_dev = device_get_binding(gpio_label);
    if (!gpio_dev) {
        LOG_ERR("GPIO device '%s' not found", gpio_label);
        return CH375_PARAM_INVALID;
    }
    LOG_INF("GPIO binding ok: %s -> %p", gpio_label, gpio_dev);

    /* allocate from static pool */
    priv = alloc_slot(uart_label, gpio_label);
    if (!priv) {
        LOG_ERR("no free CH375 adapter slots");
        return CH375_PARAM_INVALID;
    }

    priv->uart = uart_dev;
    priv->gpio = gpio_dev;
    priv->int_pin = (gpio_pin_t)int_pin;
    priv->rx_timeout_ms = 300; /* default read timeout (ms) - tune if needed */

    k_sem_init(&priv->int_sem, 0, 1);

    /* configure INT pin: input with pull-up by default */
    ret = gpio_pin_configure(priv->gpio, priv->int_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("gpio_pin_configure failed (%d)", ret);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    gpio_init_callback(&priv->gcb, ch375_gpio_callback, BIT(priv->int_pin));
    ret = gpio_add_callback(priv->gpio, &priv->gcb);
    if (ret) {
        LOG_ERR("gpio_add_callback failed (%d)", ret);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    /* Configure edge depending on HW (falling edge typical). Use EDGE_TO_ACTIVE as default. */
    ret = gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("gpio_pin_interrupt_configure failed (%d)", ret);
        gpio_remove_callback(priv->gpio, &priv->gcb);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    /* open ch375 context; ch375_openContext will store the priv pointer */
    ret = ch375_openContext((CH375_Context_t **)ctx_out,
                            z_write_cmd,
                            z_write_data,
                            z_read_data,
                            z_query_int,
                            (void *)priv);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("ch375_openContext failed (%d)", ret);
        gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_DISABLE);
        gpio_remove_callback(priv->gpio, &priv->gcb);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    LOG_INF("CH375 Zephyr adapter opened (UART=%s, GPIO=%s, INT=%u)",
            uart_label, gpio_label, (unsigned)int_pin);

    return CH375_SUCCESS;
}

int ch375_zephyr_close(CH375_Context_t *ctx)
{
    if (!ctx) return CH375_PARAM_INVALID;
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(ctx);
    if (!priv) return ch375_writeCmd_FAILD;

    /* disable interrupts and remove callback */
    gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_DISABLE);
    gpio_remove_callback(priv->gpio, &priv->gcb);

    /* close ch375 context */
    ch375_closeContext(ctx);

    /* free static slot */
    free_slot(priv);

    LOG_INF("CH375 Zephyr adapter closed");
    return CH375_SUCCESS;
}

/* ---------- STM32 register-level 9-bit UART helpers (definitions) ---------- */
#if defined(CONFIG_SOC_SERIES_STM32F4X)

/* map common label names to USART_TypeDef* */
static USART_TypeDef *get_usart_from_label(const char *label)
{
    if (!label) {
        return NULL;
    }
    /* Accept many common label variants; adjust as needed for your DTS names */
    if ((strcmp(label, "USART_2") == 0) || (strcmp(label, "UART_2") == 0) ||
        (strcmp(label, "usart2") == 0) || (strcmp(label, "USART2") == 0) ||
        (strcmp(label, "usart_2") == 0)) {
        return USART2;
    }
    if ((strcmp(label, "USART_3") == 0) || (strcmp(label, "UART_3") == 0) ||
        (strcmp(label, "usart3") == 0) || (strcmp(label, "USART3") == 0) ||
        (strcmp(label, "usart_3") == 0)) {
        return USART3;
    }
    return NULL;
}

/* Minimal helper to set M = 1 (9-bit words) and enable TE/RE.
 * We purposely do minimal touching: do not change baud if already configured.
 * Note: Caller must ensure clocks/pins are enabled (Zephyr device tree usually already did).
 */
static void configure_usart_for_9bit(USART_TypeDef *usart)
{
    if (!usart) {
        return;
    }

    /* Temporarily disable USART */
    usart->CR1 &= ~USART_CR1_UE;
    /* set M = 1 for 9-bit data length */
    usart->CR1 |= USART_CR1_M;
    /* Ensure parity disabled (we want raw 9 data bits) */
    usart->CR1 &= ~USART_CR1_PCE;
    /* Enable transmitter and receiver */
    usart->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    /* Re-enable USART */
    usart->CR1 |= USART_CR1_UE;

    /* flush flags by reading SR and DR */
    volatile uint32_t tmp = usart->SR;
    tmp = usart->DR;
    (void)tmp;
}

/* Write a single 9-bit value: value must be 0..0x1FF (9 bits) */
static int usart_write9(USART_TypeDef *usart, uint16_t value)
{
    if (!usart) {
        return -1;
    }
    /* Wait for TXE */
    uint32_t start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_TXE)) {
        if ((k_uptime_get_32() - start) > 500) { /* timeout ms */
            return -2;
        }
        k_msleep(0);
    }
    usart->DR = (uint16_t)(value & 0x1FF);
    /* Wait for TC optionally */
    start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_TC)) {
        if ((k_uptime_get_32() - start) > 500) {
            break;
        }
        k_msleep(0);
    }
    return 0;
}

/* Read a single 9-bit value into out; return 0 on success */
static int usart_read9(USART_TypeDef *usart, uint16_t *out, uint32_t timeout_ms)
{
    if (!usart || !out) {
        return -1;
    }
    uint32_t start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_RXNE)) {
        if ((k_uptime_get_32() - start) > timeout_ms) {
            return -2;
        }
        k_msleep(0);
    }
    uint16_t v = (uint16_t)(usart->DR & 0x1FF);
    *out = v;
    return 0;
}
#endif /* CONFIG_SOC_SERIES_STM32F4X */

/* ---------- Implementation of function pointers ---------- */

static int z_write_cmd(CH375_Context_t *context, uint8_t cmd)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart) {
        return ch375_writeCmd_FAILD;
    }

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        /* ensure 9-bit mode configured (idempotent) */
        configure_usart_for_9bit(usart);
        /* set 9th bit = 1 for command frames (adjust if your device uses opposite convention) */
        uint16_t txval = 0x100 | (uint16_t)cmd;
        int rc = usart_write9(usart, txval);
        return (rc == 0) ? CH375_SUCCESS : ch375_writeCmd_FAILD;
    }
#endif

    /* fallback to Zephyr UART poll out (8-bit) */
    uart_poll_out(priv->uart, cmd);
    return CH375_SUCCESS;
}

static int z_write_data(CH375_Context_t *context, uint8_t data)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart) {
        return ch375_writeCmd_FAILD;
    }

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        configure_usart_for_9bit(usart);
        /* Data frames: send 9th bit = 0 */
        uint16_t txval = (uint16_t)data & 0xFF;
        int rc = usart_write9(usart, txval);
        return (rc == 0) ? CH375_SUCCESS : ch375_writeCmd_FAILD;
    }
#endif

    uart_poll_out(priv->uart, data);
    return CH375_SUCCESS;
}

static int z_read_data(CH375_Context_t *context, uint8_t *data)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart || !data) {
        return ch375_readData_FAILD;
    }

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        uint16_t v;
        int rc = usart_read9(usart, &v, priv->rx_timeout_ms);
        if (rc == 0) {
            /* return lower 8 bits to caller; if you need the 9th bit change API */
            *data = (uint8_t)(v & 0xFF);
            return CH375_SUCCESS;
        } else {
            return ch375_readData_FAILD;
        }
    }
#endif

    /* fallback to Zephyr poll in */
    unsigned char c;
    int rc = uart_poll_in(priv->uart, &c);
    if (rc == 0) {
        *data = (uint8_t)c;
        return CH375_SUCCESS;
    }
    return ch375_readData_FAILD;
}

/* non-blocking query for interrupt - returns 0 if no int, non-zero if int pending */
static int z_query_int(CH375_Context_t *context)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv) return 0;

    if (k_sem_take(&priv->int_sem, K_NO_WAIT) == 0) {
        return 1;
    }
    return 0;
}

/* GPIO callback - called from ISR context */
static void ch375_gpio_callback(const struct device *gpiodev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    ARG_UNUSED(gpiodev);
    ch375_adapter_priv_t *priv = CONTAINER_OF(cb, ch375_adapter_priv_t, gcb);
    /* give semaphore from ISR */
    k_sem_give(&priv->int_sem);
}
