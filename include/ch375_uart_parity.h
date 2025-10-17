/* ch375_uart_parity.h - parity MARK/SPACE transport for CH375 on Zephyr
 *
 * Exposes the full struct so application code can declare static instances.
 *
 * NOTE: This implementation uses uart_configure() to toggle parity
 * between UART_CFG_PARITY_ODD (MARK approximation) and UART_CFG_PARITY_EVEN (SPACE approximation).
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ch375_parity_ctx {
    const struct device *uart_dev;
    struct k_mutex lock;
    uint32_t baud;
    uint32_t char_time_ms; /* estimated time to send one char including start/parity/stop */
    /* saved baseline uart configuration (we set this on open). */
    struct uart_config baseline_cfg;
    char dev_label[32];
};

int ch375_parity_open(struct ch375_parity_ctx *ctx, const char *uart_label, uint32_t baud);
void ch375_parity_close(struct ch375_parity_ctx *ctx);
int ch375_parity_send_cmd(struct ch375_parity_ctx *ctx, uint8_t cmd);   /* MARK */
int ch375_parity_send_data(struct ch375_parity_ctx *ctx, uint8_t data); /* SPACE */
int ch375_parity_read_byte_timeout(struct ch375_parity_ctx *ctx, uint8_t *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
