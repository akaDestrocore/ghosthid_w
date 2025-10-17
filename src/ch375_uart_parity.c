/* ch375_uart_parity.c
 *
 * Parity MARK/SPACE transport (ODD/EVEN as MARK/SPACE approximation).
 */

#include "ch375_uart_parity.h"

/* keep same implementation you had, using uart_configure, uart_poll_out, uart_fifo_read */
LOG_MODULE_REGISTER(ch375_parity, LOG_LEVEL_INF);

static uint32_t _compute_char_time_ms(uint32_t baud)
{
    if (baud == 0) baud = 115200;
    uint32_t bits = 11U; /* start + 8 data + parity + stop */
    uint32_t t_ms = (bits * 1000U + baud - 1U) / baud;
    if (t_ms < 1U) t_ms = 1U;
    return t_ms + 2U;
}

int ch375_parity_open(struct ch375_parity_ctx *ctx, const char *uart_label, uint32_t baud)
{
    if (!ctx || !uart_label) return -EINVAL;
    memset(ctx, 0, sizeof(*ctx));
    strncpy(ctx->dev_label, uart_label, sizeof(ctx->dev_label) - 1);

    ctx->uart_dev = device_get_binding(uart_label);
    if (!ctx->uart_dev) {
        LOG_ERR("device_get_binding('%s') failed", uart_label);
        return -ENODEV;
    }

    k_mutex_init(&ctx->lock);

    struct uart_config cfg = {
        .baudrate = baud ? baud : 115200,
        .data_bits = UART_CFG_DATA_BITS_8,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .parity = UART_CFG_PARITY_NONE,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };

    int rc = uart_configure(ctx->uart_dev, &cfg);
    if (rc) {
        LOG_ERR("uart_configure() failed: %d", rc);
        return -EIO;
    }
    ctx->baseline_cfg = cfg;
    ctx->baud = cfg.baudrate;
    ctx->char_time_ms = _compute_char_time_ms(ctx->baud);

    uart_irq_rx_enable(ctx->uart_dev);

    LOG_INF("ch375_parity_open %s @ %u baud char_time_ms=%u", uart_label, ctx->baud, ctx->char_time_ms);
    return 0;
}

void ch375_parity_close(struct ch375_parity_ctx *ctx)
{
    if (!ctx || !ctx->uart_dev) return;
    uart_irq_rx_disable(ctx->uart_dev);
    memset(ctx, 0, sizeof(*ctx));
}

static int _set_parity_cfg(const struct device *dev, uint32_t baud, enum uart_config_parity parity)
{
    struct uart_config cfg = {
        .baudrate = baud,
        .data_bits = UART_CFG_DATA_BITS_8,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .parity = parity,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    return uart_configure(dev, &cfg);
}

static void _wait_char_time(const struct ch375_parity_ctx *ctx)
{
    k_msleep(ctx->char_time_ms);
}

static int _send_with_parity(struct ch375_parity_ctx *ctx, uint8_t byte, enum uart_config_parity parity)
{
    if (!ctx || !ctx->uart_dev) return -EINVAL;
    int rc = 0;
    k_mutex_lock(&ctx->lock, K_FOREVER);

    rc = _set_parity_cfg(ctx->uart_dev, ctx->baud, parity);
    if (rc) {
        LOG_ERR("failed set parity %d rc=%d", parity, rc);
        (void)uart_configure(ctx->uart_dev, &ctx->baseline_cfg);
        k_mutex_unlock(&ctx->lock);
        return -EIO;
    }

    uart_poll_out(ctx->uart_dev, byte);
    _wait_char_time(ctx);

    rc = uart_configure(ctx->uart_dev, &ctx->baseline_cfg);
    if (rc) {
        LOG_WRN("warning: failed to restore baseline uart config rc=%d", rc);
    }

    k_mutex_unlock(&ctx->lock);
    return 0;
}

int ch375_parity_send_cmd(struct ch375_parity_ctx *ctx, uint8_t cmd)
{
    return _send_with_parity(ctx, cmd, UART_CFG_PARITY_ODD);
}

int ch375_parity_send_data(struct ch375_parity_ctx *ctx, uint8_t data)
{
    return _send_with_parity(ctx, data, UART_CFG_PARITY_EVEN);
}

int ch375_parity_read_byte_timeout(struct ch375_parity_ctx *ctx, uint8_t *out, uint32_t timeout_ms)
{
    if (!ctx || !ctx->uart_dev || !out) return -EINVAL;
    uint32_t start = k_uptime_get_32();
    uint8_t buf;
    int r;
    while (true) {
        r = uart_fifo_read(ctx->uart_dev, &buf, 1);
        if (r > 0) {
            *out = buf;
            return 0;
        }
        if (timeout_ms && (k_uptime_get_32() - start >= timeout_ms)) {
            return -ETIMEDOUT;
        }
        k_msleep(1);
    }
}
