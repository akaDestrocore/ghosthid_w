/* src/hid/async_report.c
 *
 * Safe asynchronous HID report sender: copies report and defers to workqueue.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include "usb/composite_hid.h"

LOG_MODULE_REGISTER(async_report, LOG_LEVEL_DBG);

struct async_report {
    struct k_work work;
    uint8_t interface_num;
    uint8_t *buf;
    size_t len;
};

static void async_report_work_handler(struct k_work *work)
{
    struct async_report *r = CONTAINER_OF(work, struct async_report, work);
    int rc = 0;

    if (!r) {
        return;
    }

    /* check USB configured flag, avoid sending when not configured */
    if (!composite_hid_is_configured()) {
        LOG_DBG("USB not configured; dropping report for iface %u", r->interface_num);
        goto out_free;
    }

    /* call the existing thread-context send (which calls usb_write) */
    rc = hid_device_send_report(r->interface_num, r->buf, r->len);
    if (rc) {
        LOG_ERR("hid_device_send_report(if=%u) rc=%d", r->interface_num, rc);
    } else {
        LOG_DBG("hid_device_send_report(if=%u) success len=%zu", r->interface_num, r->len);
    }

out_free:
    k_free(r->buf);
    k_free(r);
}

int composite_hid_send_report_async(uint8_t interface_num, const uint8_t *report, size_t len)
{
    struct async_report *r;
    uint8_t *copy;

    if (interface_num >= COMPOSITE_HID_MAX_INTERFACES) {
        return -EINVAL;
    }
    if (!report || len == 0 || len > HID_REPORT_MAX_SIZE) {
        return -EINVAL;
    }

    r = k_malloc(sizeof(*r));
    if (!r) {
        return -ENOMEM;
    }

    copy = k_malloc(len);
    if (!copy) {
        k_free(r);
        return -ENOMEM;
    }

    memcpy(copy, report, len);

    k_work_init(&r->work, async_report_work_handler);
    r->interface_num = interface_num;
    r->buf = copy;
    r->len = len;

    /* submit to system workqueue (thread context) */
    if (k_work_submit(&r->work) != 0) {
        k_free(copy);
        k_free(r);
        return -EIO;
    }

    return 0;
}
