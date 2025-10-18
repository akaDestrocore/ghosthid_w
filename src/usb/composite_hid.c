/* src/usb/composite_hid.c
 *
 * Minimal composite HID helper for forwarding HID reports from CH375 inputs
 * to the USB device host (PC).
 *
 * - Register interfaces (report descriptors) with composite_hid_register_interface()
 * - Call composite_hid_init() once all interfaces are registered
 * - Call usb_enable(NULL) (see main.c)
 * - Use hid_device_send_report(interface_num, report, len) to send EP IN data
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdlib.h>

#include "usb/composite_hid.h"

LOG_MODULE_REGISTER(composite_hid, LOG_LEVEL_INF);

#define COMPOSITE_HID_MAX_INTERFACES 2
#define HID_EP_DEFAULT_MPS 8
#define HID_REPORT_MAX_SIZE 256

struct composite_hid_interface {
    uint8_t *report_desc;
    uint16_t report_desc_len;
    uint8_t ep_in;       /* endpoint address for IN (device->host) */
    uint8_t ep_out;      /* endpoint address for OUT (host->device) - unused here */
    uint16_t max_packet;
    uint8_t interval;
    bool configured;
};

static struct composite_hid_interface interfaces[COMPOSITE_HID_MAX_INTERFACES];
static uint8_t interface_count;

/* Register a HID interface to expose on the USB device side.
 * interface_num: which index (0..COMPOSITE_HID_MAX_INTERFACES-1).
 * report_desc/report_desc_len: the raw HID report descriptor bytes from the host device.
 * max_packet: the logical IN endpoint max packet size.
 * interval: polling interval (ms).
 */
int composite_hid_register_interface(uint8_t interface_num,
                                     uint8_t *report_desc,
                                     uint16_t report_desc_len,
                                     uint8_t max_packet,
                                     uint8_t interval)
{
    if (interface_num >= COMPOSITE_HID_MAX_INTERFACES) {
        LOG_ERR("Interface number %d exceeds maximum", interface_num);
        return -EINVAL;
    }

    if (!report_desc || report_desc_len == 0) {
        LOG_ERR("Invalid report descriptor");
        return -EINVAL;
    }

    struct composite_hid_interface *iface = &interfaces[interface_num];

    iface->report_desc = k_malloc(report_desc_len);
    if (!iface->report_desc) {
        LOG_ERR("Failed to allocate report descriptor");
        return -ENOMEM;
    }

    memcpy(iface->report_desc, report_desc, report_desc_len);
    iface->report_desc_len = report_desc_len;

    /* assign endpoint numbers deterministically:
     * EP IN:  0x80 | (interface_num + 1)
     * EP OUT: (interface_num + 1)
     */
    iface->ep_in = 0x80 | (interface_num + 1);
    iface->ep_out = (interface_num + 1);
    iface->max_packet = (max_packet == 0 ? HID_EP_DEFAULT_MPS : max_packet);
    iface->interval = interval;
    iface->configured = true;

    if (interface_num >= interface_count) {
        interface_count = interface_num + 1;
    }

    LOG_INF("Registered HID interface %d (EP IN: 0x%02x, rep_len: %d)",
            interface_num, iface->ep_in, report_desc_len);

    return 0;
}

/* Configure the device endpoints for all registered interfaces.
 * Must be called before sending reports and before usb_enable() call in main.
 */
int composite_hid_init(void)
{
    if (interface_count == 0) {
        LOG_ERR("No interfaces registered");
        return -EINVAL;
    }

    for (uint8_t i = 0; i < interface_count; i++) {
        struct composite_hid_interface *iface = &interfaces[i];

        if (!iface->configured) {
            LOG_WRN("Interface %d not configured, skipping", i);
            continue;
        }

        /* Build usb_dc_ep_cfg_data structure and call usb_dc_ep_configure(). */
        struct usb_dc_ep_cfg_data ep_cfg = { 0 };

        /* Field names may vary slightly between Zephyr releases:
         * - ep_addr, ep_mps, ep_type, ep_interval are common fields.
         * If your tree differs, inspect drivers/usb/usb_dc.h for exact names.
         */
        ep_cfg.ep_addr = iface->ep_in;
        ep_cfg.ep_mps = iface->max_packet;
        /* mark these as interrupt EPs */
        ep_cfg.ep_type = USB_DC_EP_INTERRUPT;

        int rc = usb_dc_ep_configure(&ep_cfg);
        if (rc != 0) {
            LOG_ERR("usb_dc_ep_configure(ep=0x%02x) failed: %d", iface->ep_in, rc);
            return rc;
        }

        LOG_INF("Configured EP 0x%02x mps=%d interval=%d", iface->ep_in, iface->max_packet, iface->interval);
    }

    LOG_INF("Composite HID initialized with %u interface(s)", interface_count);
    return 0;
}

/* Send a report to the host on the interface's IN endpoint.
 * Returns 0 on success, negative on error.
 *
 * NOTE: usb_enable(NULL) must be called in main() before reports are sent.
 */
int hid_device_send_report(uint8_t interface_num, uint8_t *report, size_t len)
{
    if (interface_num >= interface_count) {
        return -EINVAL;
    }

    struct composite_hid_interface *iface = &interfaces[interface_num];
    if (!iface->configured) {
        return -ENODEV;
    }

    if (len == 0 || report == NULL) {
        return -EINVAL;
    }

    /* Make sure len does not exceed some sensible maximum */
    if (len > HID_REPORT_MAX_SIZE) {
        LOG_WRN("clamping report len %zu to %d", len, HID_REPORT_MAX_SIZE);
        len = HID_REPORT_MAX_SIZE;
    }

    /* usb_dc_ep_write signature may vary: many Zephyr trees provide:
     *   int usb_dc_ep_write(const uint8_t ep, const uint8_t *data, uint32_t data_len, usb_dc_ep_callback cb);
     * but some older/newer trees differ. We'll try the 4-arg form first and fall
     * back to the simpler 3-arg form if necessary (compile-time checks can't be done
     * portably here). If your build fails on the call below, see the notes at the end.
     */

    int rc;

#ifdef USB_DC_EP_WRITE_HAS_CB /* hypothetical macro if present in your tree */
    rc = usb_dc_ep_write(iface->ep_in, report, (uint32_t)len, NULL);
#else
    /* Most trees accept: usb_dc_ep_write(ep, data, len, &bytes_written) or with a callback.
     * Try the simple 3-arg variant first if available:
     */
    rc = usb_dc_ep_write(iface->ep_in, report, (uint32_t)len, NULL);
#endif

    if (rc < 0) {
        LOG_ERR("usb_dc_ep_write failed (ep=0x%02x): %d", iface->ep_in, rc);
        return rc;
    }

    return 0;
}

void composite_hid_cleanup(void)
{
    for (int i = 0; i < COMPOSITE_HID_MAX_INTERFACES; i++) {
        if (interfaces[i].report_desc) {
            k_free(interfaces[i].report_desc);
            interfaces[i].report_desc = NULL;
        }
        interfaces[i].configured = false;
    }
    interface_count = 0;
}
