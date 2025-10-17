/*
 * usb_hid_forwarder.c
 *
 * Background thread that:
 *  - waits until devices are found by CH375 (host)
 *  - opens the usbhid wrappers
 *  - reads reports from CH375-host side
 *  - sends them over local USB device HID to the PC
 *
 * The only piece left for you to implement is usb_hid_send_report()
 * which must call into Zephyr's USB HID device class transmit API (depends on Zephyr version).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "ch375.h"
#include "ch375_usbhost.h"
#include "hid/usbhid.h"
#include "hid/hid_mouse.h"
#include "hid/hid_keyboard.h"

LOG_MODULE_REGISTER(usb_forwarder, LOG_LEVEL_INF);

/* forward declaration - this is the single Zephyr USB HID API hook you must implement
 *
 * Parameters:
 *   interface_idx  - which composite HID interface to send to (0..N). In HAL firmware they used index==dev slot.
 *   report         - pointer to the HID report bytes to send
 *   report_len     - length of the report (bytes)
 *
 * Implement this function to call Zephyr's USB HID device endpoint write.
 * See the NOTES after this file for Zephyr API guidance.
 */
int usb_hid_send_report(uint8_t interface_idx, uint8_t *report, uint32_t report_len)
{
    /* TODO: implement using Zephyr's USB HID device API for your Zephyr version
     *
     * Examples (depending on your Zephyr version / stack):
     *  - New USB device_next API: call the HID endpoint write function registered
     *    for your composite interface. The function name differs between versions.
     *  - Old stack: call something like `usb_hid_int_ep_write(report, report_len)` or
     *    a device-specific wrapper.
     *
     * For now we log and return success to let the rest of code compile and run.
     */
    LOG_DBG("usb_hid_send_report iface=%u len=%u", interface_idx, report_len);
    ARG_UNUSED(interface_idx);
    ARG_UNUSED(report);
    ARG_UNUSED(report_len);
    return 0;
}

/* thread to poll opened HID devices and forward their reports */
#define STACK_SIZE 4096
#define PRIORITY 7
static K_THREAD_STACK_DEFINE(forwarder_stack, STACK_SIZE);
static struct k_thread forwarder_thread_data;

void usb_hid_forwarder_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("usb_hid_forwarder thread started");

    while (1) {
        /* Try to open all CH375-attached devices into host mode and HID wrappers */
        for (int i = 0; i < 2; i++) {
            CH375_Context_t *ctx = ch375_zephyr_get_context(i);
            if (!ctx) {
                LOG_ERR("ctx %d NULL", i);
                continue;
            }
            USB_Device_t udev;
            USBHID_Device_t hiddev;
            memset(&udev, 0, sizeof(udev));
            memset(&hiddev, 0, sizeof(hiddev));

            int ret = ch375_hostUdevOpen(ctx, &udev);
            if (ret != CH375_HST_ERRNO_SUCCESS) {
                LOG_DBG("ch375_hostUdevOpen idx=%d ret=%d", i, ret);
                continue;
            }

            ret = usbhid_open(&udev, 0, &hiddev); /* interface 0 assumed */
            if (ret != USBHID_ERRNO_SUCCESS) {
                LOG_ERR("usbhid_open failed idx=%d ret=%d", i, ret);
                ch375_hostUdevClose(&udev);
                continue;
            }

            LOG_INF("Device %d opened: hid_type=%d report_len=%u", i, hiddev.hid_type, hiddev.report_length);

            /* allocate report buffer per usbhid_alloc_report_buffer() is done in the class opens. Now loop and forward */
            while (1) {
                int rret;
                if (hiddev.hid_type == USBHID_TYPE_MOUSE) {
                    HIDMouse_t mouse;
                    memset(&mouse, 0, sizeof(mouse));
                    if (hid_mouse_open(&hiddev, &mouse) != USBHID_ERRNO_SUCCESS) {
                        LOG_ERR("hid_mouse_open failed");
                        break;
                    }
                    /* per-loop fetch and forward */
                    rret = hid_mouse_fetch_report(&mouse);
                    if (rret == USBHID_ERRNO_SUCCESS) {
                        uint8_t *buf;
                        uint32_t len;
                        if (usbhid_get_report_buffer(&hiddev, &buf, &len, USBHID_NOW) == USBHID_ERRNO_SUCCESS) {
                            usb_hid_send_report(i, buf, len);
                        }
                    } else {
                        /* disconnected or no device */
                        break;
                    }
                    hid_mouse_close(&mouse);
                } else if (hiddev.hid_type == USBHID_TYPE_KEYBOARD) {
                    HIDKeyboard_t keyboard;
                    memset(&keyboard, 0, sizeof(keyboard));
                    if (hid_keyboard_open(&hiddev, &keyboard) != USBHID_ERRNO_SUCCESS) {
                        LOG_ERR("hid_keyboard_open failed");
                        break;
                    }
                    rret = hid_keyboard_fetch_report(&keyboard);
                    if (rret == USBHID_ERRNO_SUCCESS) {
                        uint8_t *buf;
                        uint32_t len;
                        if (usbhid_get_report_buffer(&hiddev, &buf, &len, USBHID_NOW) == USBHID_ERRNO_SUCCESS) {
                            usb_hid_send_report(i, buf, len);
                        }
                    } else {
                        break;
                    }
                    hid_keyboard_close(&keyboard);
                } else {
                    LOG_WRN("unsupported hid type %d", hiddev.hid_type);
                    break;
                }
                k_msleep(5); /* small throttle */
            }

            LOG_INF("closing device idx=%d", i);
            usbhid_close(&hiddev);
            ch375_hostUdevClose(&udev);
        } /* for devices */

        /* if no devices, wait a bit before trying again */
        k_msleep(200);
    }
}

/* start the forwarder thread (call from main) */
void usb_hid_forwarder_start(void)
{
    k_thread_create(&forwarder_thread_data, forwarder_stack,
                    K_THREAD_STACK_SIZEOF(forwarder_stack),
                    usb_hid_forwarder_thread,
                    NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
}
