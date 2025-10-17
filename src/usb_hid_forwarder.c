/* src/usb_hid_forwarder.c
 *
 * Background thread that:
 *  - waits until devices are found by CH375
 *  - opens the usbhid wrappers
 *  - reads reports from CH375-host side
 *  - applies auto_gun_press modifications to mouse
 *  - sends modified reports over USB device HID to the PC
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
#include "usb_hid_device.h"
#include "auto_gun_press.h"

LOG_MODULE_REGISTER(usb_forwarder, LOG_LEVEL_INF);

/* External glue functions */
extern CH375_Context_t *ch375_zephyr_get_context(int idx);

/* AGP context for mouse modification */
static AGPContext *g_agp_ctx = NULL;
static bool g_agp_active = false;
static bool g_mouse_button_pressed = false;

/* Button state tracking for toggling AGP modes */
#define TOGGLE_BUTTON_NUM 3  /* Middle mouse button to toggle AGP */
#define MODE_BUTTON_NUM 4    /* Extra button to cycle weapon modes */
static bool g_last_toggle_state = false;
static bool g_last_mode_state = false;

/* Mouse report structure (8 bytes) */
typedef struct {
    uint16_t buttons;  /* 16-bit button state */
    int16_t x;         /* X movement */
    int16_t y;         /* Y movement */
    int8_t wheel;      /* Wheel */
    uint8_t padding;   /* Padding */
} __attribute__((packed)) MouseReport;

/* Keyboard report structure (8 bytes) */
typedef struct {
    uint8_t modifiers;  /* Modifier keys */
    uint8_t reserved;   /* Reserved */
    uint8_t keys[6];    /* Key codes */
} __attribute__((packed)) KeyboardReport;

/* Initialize AGP module */
static int init_agp(void)
{
    int ret = agp_open(&g_agp_ctx);
    if (ret < 0) {
        LOG_ERR("Failed to open AGP context");
        return ret;
    }
    LOG_INF("AGP initialized with AK47 pattern");
    return 0;
}

/* Apply AGP modifications to mouse report */
static void apply_agp_to_mouse(uint8_t *report, uint32_t report_len)
{
    if (!g_agp_ctx || !g_agp_active || report_len != 8) {
        return;
    }

    MouseReport *mr = (MouseReport *)report;
    
    /* Check if left button is pressed (bit 0) */
    bool left_button = (mr->buttons & 0x01) != 0;
    
    /* Detect button press transition */
    if (left_button && !g_mouse_button_pressed) {
        /* Button just pressed - restart AGP sequence */
        agp_restart(g_agp_ctx);
        g_mouse_button_pressed = true;
        LOG_DBG("Left button pressed - AGP sequence started");
    } else if (!left_button && g_mouse_button_pressed) {
        /* Button released */
        g_mouse_button_pressed = false;
        LOG_DBG("Left button released");
    }
    
    /* Apply recoil compensation if button is pressed */
    if (g_mouse_button_pressed) {
        AGPData agp_data;
        int ret = agp_get_data(g_agp_ctx, &agp_data);
        if (ret == 0) {
            /* Add recoil compensation to mouse movement */
            mr->x += (int16_t)agp_data.x;
            mr->y += (int16_t)agp_data.y;
            LOG_DBG("AGP applied: x=%d, y=%d (original x=%d, y=%d)", 
                   (int)agp_data.x, (int)agp_data.y, 
                   (int)(mr->x - agp_data.x), (int)(mr->y - agp_data.y));
        }
    }
}

/* Handle special button presses for AGP control */
static void handle_agp_controls(uint8_t *report, uint32_t report_len)
{
    if (!g_agp_ctx || report_len != 8) {
        return;
    }

    MouseReport *mr = (MouseReport *)report;
    
    /* Check toggle button (middle click) */
    bool toggle_pressed = (mr->buttons & (1 << TOGGLE_BUTTON_NUM)) != 0;
    if (toggle_pressed && !g_last_toggle_state) {
        /* Toggle AGP on/off */
        g_agp_active = !g_agp_active;
        LOG_INF("AGP %s", g_agp_active ? "ENABLED" : "DISABLED");
    }
    g_last_toggle_state = toggle_pressed;
    
    /* Check mode button (extra button 1) */
    bool mode_pressed = (mr->buttons & (1 << MODE_BUTTON_NUM)) != 0;
    if (mode_pressed && !g_last_mode_state) {
        /* Cycle through weapon patterns */
        static uint32_t current_mode = AGP_COLLECT_IDX_AK47;
        current_mode = (current_mode % 2) + 1;  /* Toggle between 1 (AK47) and 2 (M4A4) */
        agp_set_collect(g_agp_ctx, current_mode);
        LOG_INF("Weapon mode changed to %s", 
               current_mode == AGP_COLLECT_IDX_AK47 ? "AK47" : "M4A4");
    }
    g_last_mode_state = mode_pressed;
}

/* Thread to poll opened HID devices and forward their reports */
#define STACK_SIZE 8192
#define PRIORITY 7
static K_THREAD_STACK_DEFINE(forwarder_stack, STACK_SIZE);
static struct k_thread forwarder_thread_data;

void usb_hid_forwarder_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("usb_hid_forwarder thread started");

    /* Initialize USB HID device */
    int ret = usb_hid_device_init();
    if (ret) {
        LOG_ERR("Failed to initialize USB HID device: %d", ret);
        return;
    }

    /* Initialize AGP */
    ret = init_agp();
    if (ret) {
        LOG_WRN("AGP initialization failed, continuing without AGP");
    }

    while (1) {
        /* Try to open all CH375-attached devices */
        for (int i = 0; i < 2; i++) {
            CH375_Context_t *ctx = ch375_zephyr_get_context(i);
            if (!ctx) {
                continue;
            }
            
            USB_Device_t udev;
            USBHID_Device_t hiddev;
            memset(&udev, 0, sizeof(udev));
            memset(&hiddev, 0, sizeof(hiddev));

            ret = ch375_hostUdevOpen(ctx, &udev);
            if (ret != CH375_HST_ERRNO_SUCCESS) {
                k_msleep(100);
                continue;
            }

            ret = usbhid_open(&udev, 0, &hiddev);
            if (ret != USBHID_ERRNO_SUCCESS) {
                LOG_ERR("usbhid_open failed idx=%d ret=%d", i, ret);
                ch375_hostUdevClose(&udev);
                continue;
            }

            LOG_INF("Device %d opened: hid_type=%d report_len=%u", 
                   i, hiddev.hid_type, hiddev.report_length);

            /* Process reports in a loop */
            while (1) {
                int rret;
                uint8_t *buf;
                uint32_t len;
                
                if (hiddev.hid_type == USBHID_TYPE_MOUSE) {
                    HIDMouse_t mouse;
                    memset(&mouse, 0, sizeof(mouse));
                    
                    if (hid_mouse_open(&hiddev, &mouse) != USBHID_ERRNO_SUCCESS) {
                        LOG_ERR("hid_mouse_open failed");
                        break;
                    }
                    
                    rret = hid_mouse_fetch_report(&mouse);
                    if (rret == USBHID_ERRNO_SUCCESS) {
                        if (usbhid_get_report_buffer(&hiddev, &buf, &len, USBHID_NOW) 
                            == USBHID_ERRNO_SUCCESS) {
                            /* Handle AGP control buttons */
                            handle_agp_controls(buf, len);
                            /* Apply AGP modifications */
                            apply_agp_to_mouse(buf, len);
                            /* Send modified report to PC */
                            usb_hid_send_report(i, buf, len);
                        }
                    } else if (rret == USBHID_ERRNO_NO_DEV || 
                              rret == USBHID_ERRNO_DEV_DISCONNECT) {
                        LOG_INF("Mouse device %d disconnected", i);
                        hid_mouse_close(&mouse);
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
                        if (usbhid_get_report_buffer(&hiddev, &buf, &len, USBHID_NOW) 
                            == USBHID_ERRNO_SUCCESS) {
                            /* Forward keyboard report unmodified */
                            usb_hid_send_report(i, buf, len);
                        }
                    } else if (rret == USBHID_ERRNO_NO_DEV || 
                              rret == USBHID_ERRNO_DEV_DISCONNECT) {
                        LOG_INF("Keyboard device %d disconnected", i);
                        hid_keyboard_close(&keyboard);
                        break;
                    }
                    
                    hid_keyboard_close(&keyboard);
                    
                } else {
                    LOG_WRN("unsupported hid type %d", hiddev.hid_type);
                    break;
                }
                
                k_msleep(1);  /* Small throttle - 1ms poll rate */
            }

            LOG_INF("closing device idx=%d", i);
            usbhid_close(&hiddev);
            ch375_hostUdevClose(&udev);
        }

        k_msleep(200);
    }
}

/* Start the forwarder thread (call from main) */
void usb_hid_forwarder_start(void)
{
    k_thread_create(&forwarder_thread_data, forwarder_stack,
                    K_THREAD_STACK_SIZEOF(forwarder_stack),
                    usb_hid_forwarder_thread,
                    NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&forwarder_thread_data, "hid_forwarder");
}