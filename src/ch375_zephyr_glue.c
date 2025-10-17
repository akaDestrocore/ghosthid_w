/* src/ch375_zephyr_glue.c
 *
 * Zephyr glue for CH375 modules using ch375_parity as UART transport
 *
 * Assumptions:
 *  - Two CH375 modules (idx 0 and 1)
 *  - CH375A uses USART2 and INT on GPIOE pin 14
 *  - CH375B uses USART3 and INT on GPIOE pin 15
 *
 * If your wiring differs, change the CH375_* macros below.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <assert.h>

#include "ch375.h"
#include "ch375_interface.h"
#include "ch375_uart_parity.h"   /* provides ch375_parity_ctx and APIs */

LOG_MODULE_REGISTER(ch375_zephyr_glue, LOG_LEVEL_INF);

/* ---- CONFIG: edit if your wiring/labels differ ---- */

/* UART device labels in Zephyr - must match device tree (board overlay) */
#ifndef CH375_UART_LABEL_A
#define CH375_UART_LABEL_A "USART_2"
#endif
#ifndef CH375_UART_LABEL_B
#define CH375_UART_LABEL_B "USART_3"
#endif

/* GPIO port / pins for INT (user told me these) */
#define CH375_INT_GPIO_LABEL "GPIOE"
#define CH375A_INT_PIN 14
#define CH375B_INT_PIN 15

/* Use the ch375 parity baud as default init baud (parity layer sets actual baud we ask) */
#ifndef CH375_DEFAULT_BAUD
#define CH375_DEFAULT_BAUD 9600
#endif

/* ---- end CONFIG ---- */

struct ch375_zephyr_priv {
    /* parity transport context */
    struct ch375_parity_ctx parity;
    /* gpio + callback */
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    gpio_pin_t int_pin;
    atomic_t irq_set; /* 1 when INT asserted (active low -> gpio reads 0) */
    /* reference to parent CH375 context (for debug) */
    CH375_Context_t *ctx;
};

static CH375_Context_t *g_ctx_arr[2] = { NULL, NULL };
static struct ch375_zephyr_priv g_privs[2];

/* Forward declarations matching ch375 expected callbacks */
static int zephyr_write_cmd(CH375_Context_t *context, uint8_t cmd);
static int zephyr_write_data(CH375_Context_t *context, uint8_t data);
static int zephyr_read_data(CH375_Context_t *context, uint8_t *data);
static int zephyr_query_int(CH375_Context_t *context);

/* GPIO callback: called on edge; set atomic irq flag according to pin level.
 * CH375 INT is active low: INT=0 means there is an interrupt pending.
 */
static void ch375_int_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    struct ch375_zephyr_priv *p = CONTAINER_OF(cb, struct ch375_zephyr_priv, gpio_cb);
    int val = gpio_pin_get(p->gpio_dev, p->int_pin);
    /* active-low: set flag if val == 0 */
    atomic_set(&p->irq_set, (val == 0) ? 1 : 0);
}

/* Lookup priv by CH375_Context_t* (we stored it when opening) */
static struct ch375_zephyr_priv *priv_from_ctx(CH375_Context_t *ctx)
{
    if (!ctx) return NULL;
    for (int i = 0; i < 2; ++i) {
        if (g_ctx_arr[i] == ctx) return &g_privs[i];
    }
    return NULL;
}

/* ch375 callback implementations - use parity API */
static int zephyr_write_cmd(CH375_Context_t *context, uint8_t cmd)
{
    struct ch375_zephyr_priv *p = priv_from_ctx(context);
    if (!p) return CH375_ERROR;
    if (ch375_parity_send_cmd(&p->parity, cmd) != 0) return CH375_ERROR;
    return CH375_SUCCESS;
}

static int zephyr_write_data(CH375_Context_t *context, uint8_t data)
{
    struct ch375_zephyr_priv *p = priv_from_ctx(context);
    if (!p) return CH375_ERROR;
    if (ch375_parity_send_data(&p->parity, data) != 0) return CH375_ERROR;
    return CH375_SUCCESS;
}

/* read a single byte from parity transport with a small timeout (ms) */
static int zephyr_read_data(CH375_Context_t *context, uint8_t *data)
{
    struct ch375_zephyr_priv *p = priv_from_ctx(context);
    if (!p || !data) return CH375_ERROR;
    int rc = ch375_parity_read_byte_timeout(&p->parity, data, 200); /* 200ms read timeout */
    return (rc == 0) ? CH375_SUCCESS : CH375_ERROR;
}

static int zephyr_query_int(CH375_Context_t *context)
{
    struct ch375_zephyr_priv *p = priv_from_ctx(context);
    if (!p) return 0;
    return (atomic_get(&p->irq_set) != 0) ? 1 : 0;
}

/* Public: return context pointer for forwarder/main */
CH375_Context_t *ch375_zephyr_get_context(int idx)
{
    if (idx < 0 || idx >= ARRAY_SIZE(g_ctx_arr)) return NULL;
    return g_ctx_arr[idx];
}

/* ch375_zephyr_init - initialize parity transports and ch375 contexts
 * Must be called once from main (before ch375_hostInit etc).
 */
int ch375_zephyr_init(void)
{
    int rc;
    const char *uart_label;
    const char *gpio_label = CH375_INT_GPIO_LABEL;

    /* zero structures */
    memset(g_privs, 0, sizeof(g_privs));
    memset(g_ctx_arr, 0, sizeof(g_ctx_arr));

    for (int i = 0; i < 2; ++i) {
        struct ch375_zephyr_priv *p = &g_privs[i];

        uart_label = (i == 0) ? CH375_UART_LABEL_A : CH375_UART_LABEL_B;
        p->int_pin = (i == 0) ? CH375A_INT_PIN : CH375B_INT_PIN;

        /* open parity transport using your existing API */
        rc = ch375_parity_open(&p->parity, uart_label, CH375_DEFAULT_BAUD);
        if (rc != 0) {
            LOG_ERR("ch375_parity_open(%s) failed: %d", uart_label, rc);
            return -ENODEV;
        }

        /* configure GPIO device for INT line */
        p->gpio_dev = device_get_binding(gpio_label);
        if (!p->gpio_dev) {
            LOG_ERR("GPIO device '%s' not found", gpio_label);
            return -ENODEV;
        }

        /* configure pin as input with pull-up (CH375 INT is active low) */
        rc = gpio_pin_configure(p->gpio_dev, p->int_pin, GPIO_INPUT | GPIO_PULL_UP);
        if (rc != 0) {
            LOG_ERR("gpio_pin_configure(%s,%u) failed %d", gpio_label, p->int_pin, rc);
            return rc;
        }

        /* initialize atomic flag with current pin state */
        int val = gpio_pin_get(p->gpio_dev, p->int_pin);
        atomic_set(&p->irq_set, (val == 0) ? 1 : 0);

        /* attach callback (falling & rising so we see both assert and deassert) */
        gpio_init_callback(&p->gpio_cb, ch375_int_gpio_cb, BIT(p->int_pin));
        rc = gpio_add_callback(p->gpio_dev, &p->gpio_cb);
        if (rc != 0) {
            LOG_ERR("gpio_add_callback failed %d", rc);
            return rc;
        }
        rc = gpio_pin_interrupt_configure(p->gpio_dev, p->int_pin, GPIO_INT_EDGE_BOTH);
        if (rc != 0) {
            LOG_ERR("gpio_pin_interrupt_configure failed %d", rc);
            return rc;
        }

        /* create CH375 context via upstream API ch375_openContext.
         * The expected signature of ch375_openContext is:
         *   int ch375_openContext(CH375_Context_t **ctx,
         *                         ch375_write_cmd_cb,
         *                         ch375_write_data_cb,
         *                         ch375_read_data_cb,
         *                         ch375_query_int_cb,
         *                         void *priv);
         *
         * The implementation in your ch375.c (original repo) uses such pattern.
         */
        CH375_Context_t *ctx = NULL;
        rc = ch375_openContext(&ctx,
                               zephyr_write_cmd,
                               zephyr_write_data,
                               zephyr_read_data,
                               zephyr_query_int,
                               (void *)p);
        if (rc != CH375_SUCCESS || ctx == NULL) {
            LOG_ERR("ch375_openContext failed for idx %d rc=%d", i, rc);
            return -EIO;
        }
        p->ctx = ctx;
        g_ctx_arr[i] = ctx;
    }

    LOG_INF("ch375_zephyr_init: initialized 2 contexts");
    return 0;
}
