#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "ch375.h"
#include "ch375_uart_parity.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* Your chosen pins */
#define CH375A_UART_LABEL "UART_2"   /* per overlay alias */
#define CH375B_UART_LABEL "UART_3"

#define CH375_GPIO_LABEL  "GPIOE"
#define CH375A_INT_PIN    14
#define CH375B_INT_PIN    15

/* Initial baud for detection (9600), then working baud e.g. 115200 */
#define CH375_DETECT_BAUD 9600
#define CH375_WORK_BAUD   115200

void main(void)
{
    int rc;

    LOG_INF("starting CH375-bridge on Zephyr");

    /* Initialize two devices */
    rc = ch375_zephyr_device_init(0, "CH375A", CH375A_UART_LABEL, CH375_GPIO_LABEL, CH375A_INT_PIN, CH375_DETECT_BAUD);
    if (rc) {
        LOG_ERR("ch375A init failed %d", rc);
        return;
    }
    rc = ch375_zephyr_device_init(1, "CH375B", CH375B_UART_LABEL, CH375_GPIO_LABEL, CH375B_INT_PIN, CH375_DETECT_BAUD);
    if (rc) {
        LOG_ERR("ch375B init failed %d", rc);
        return;
    }

    /* call ch375_hostInit for both contexts (this configures the device into host mode and sets target baud) */
    for (int i = 0; i < 2; i++) {
        CH375_Context_t *ctx = ch375_zephyr_get_context(i);
        if (!ctx) {
            LOG_ERR("ctx %d NULL", i);
            continue;
        }
        rc = ch375_hostInit(ctx, CH375_DETECT_BAUD);
        if (rc != 0) {
            LOG_ERR("ch375_hostInit failed for idx %d: %d", i, rc);
            continue;
        }
        /* reconfigure the UART to working baud */
        ch375_zephyr_set_baud(i, CH375_WORK_BAUD);
    }

    LOG_INF("Initialization complete — starting forwarder thread");

    /* spawn forwarder thread */
    /* forwarder is implemented in src/usb_hid_forwarder.c */
    extern void usb_hid_forwarder_start(void);
    usb_hid_forwarder_start();

    /* main can sleep or do background tasks; forwarder runs */
    while (1) {
        k_msleep(1000);
    }
}
