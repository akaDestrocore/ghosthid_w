#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/version.h>
#include <string.h>

LOG_MODULE_REGISTER(hid_bridge, LOG_LEVEL_INF);

/* CH375 and HID APIs */
#include "ch375_zephyr_adapter.h"
#include "ch375_interface.h"
#include "ch375_usbhost.h"
#include "ch375.h"
#include "hid/usbhid.h"
#include "hid/hid_mouse.h"
#include "hid/hid_keyboard.h"
#include "hid_bridge.h"
#include "usbd_core.h"
#include "composite_hid.h"

/* composite wrapper */
extern int composite_hid_send_report(uint8_t interface_idx, uint8_t *report, uint16_t len);
extern uint8_t USBD_COMPOSITE_HID_InterfaceRegister(uint8_t, uint8_t*, uint8_t*, uint16_t, uint8_t, uint8_t);
extern uint8_t USBD_COMPOSITE_HID_Init(void);


/* USB init from usbd_conf.c */
extern void MX_USB_DEVICE_Init(void);


extern USBD_HandleTypeDef hUsbDeviceFS;

/* Simple 3-button mouse report (3 bytes): buttons, dx, dy */
static const uint8_t composite_mouse_report_desc[] = {
  0x05, 0x01,        /* Usage Page (Generic Desktop) */
  0x09, 0x02,        /* Usage (Mouse) */
  0xA1, 0x01,        /* Collection (Application) */
    0x09, 0x01,      /*  Usage (Pointer) */
    0xA1, 0x00,      /*  Collection (Physical) */
      0x05, 0x09,    /*   Usage Page (Button) */
      0x19, 0x01,    /*   Usage Minimum (1) */
      0x29, 0x03,    /*   Usage Maximum (3) */
      0x15, 0x00,    /*   Logical Minimum (0) */
      0x25, 0x01,    /*   Logical Maximum (1) */
      0x95, 0x03,    /*   Report Count (3) */
      0x75, 0x01,    /*   Report Size (1) */
      0x81, 0x02,    /*   Input (Data,Var,Abs) -- Buttons */
      0x95, 0x01,    /*   Report Count (1) */
      0x75, 0x05,    /*   Report Size (5) */
      0x81, 0x03,    /*   Input (Const,Var,Abs) -- Padding */
      0x05, 0x01,    /*   Usage Page (Generic Desktop) */
      0x09, 0x30,    /*   Usage (X) */
      0x09, 0x31,    /*   Usage (Y) */
      0x15, 0x81,    /*   Logical Minimum (-127) */
      0x25, 0x7F,    /*   Logical Maximum (127) */
      0x75, 0x08,    /*   Report Size (8) */
      0x95, 0x02,    /*   Report Count (2) */
      0x81, 0x06,    /*   Input (Data,Var,Rel) -- X,Y */
    0xC0,
  0xC0
};


/* Simple keyboard (boot) report descriptor: modifiers + reserved + 6 keycodes */
static const uint8_t composite_keyboard_report_desc[] = {
  0x05, 0x01,        /* Usage Page (Generic Desktop) */
  0x09, 0x06,        /* Usage (Keyboard) */
  0xA1, 0x01,        /* Collection (Application) */
    0x05, 0x07,      /*  Usage Page (Keyboard) */
    0x19, 0xE0,      /*  Usage Minimum (224) */
    0x29, 0xE7,      /*  Usage Maximum (231) */
    0x15, 0x00,      /*  Logical Minimum (0) */
    0x25, 0x01,      /*  Logical Maximum (1) */
    0x75, 0x01,      /*  Report Size (1) */
    0x95, 0x08,      /*  Report Count (8) */
    0x81, 0x02,      /*  Input (Data,Var,Abs) -- Modifier byte */
    0x95, 0x01,      /*  Report Count (1) */
    0x75, 0x08,      /*  Report Size (8) */
    0x81, 0x03,      /*  Input (Const) -- Reserved */
    0x95, 0x06,      /*  Report Count (6) */
    0x75, 0x08,      /*  Report Size (8) */
    0x15, 0x00,      /*  Logical Minimum (0) */
    0x25, 0x65,      /*  Logical Maximum (101) */
    0x05, 0x07,      /*  Usage Page (Keyboard) */
    0x19, 0x00,      /*  Usage Minimum (0) */
    0x29, 0x65,      /*  Usage Maximum (101) */
    0x81, 0x00,      /*  Input (Data,Ary,Abs) -- Keys */
  0xC0
};

/* Configure to match your board */
#define CH375_A_UART_LABEL  "USART_2"
#define CH375_A_GPIO_LABEL  "GPIOE"
#define CH375_A_INT_PIN     14U

#define CH375_B_UART_LABEL  "USART_3"
#define CH375_B_GPIO_LABEL  "GPIOE"
#define CH375_B_INT_PIN     15U

#define DT_ALIAS_ch375_uart_a DT_ALIAS(ch375_uart_a)
#define DT_ALIAS_ch375_uart_b DT_ALIAS(ch375_uart_b)
#define DT_ALIAS_ch375_gpio   DT_ALIAS(ch375_gpio)

/* msgq for TX */
#define MAX_REPORT_SIZE 64
#define MSGQ_MSG_COUNT 32
#define MSGQ_ALIGN 4
typedef struct {
    uint8_t if_idx;
    uint16_t len;
    uint8_t data[MAX_REPORT_SIZE];
} hid_msg_t;

K_MSGQ_DEFINE(hid_tx_msgq, sizeof(hid_msg_t), MSGQ_MSG_COUNT, MSGQ_ALIGN);

static int enqueue_report_from_usbhid(USBHID_Device_t *uhdev, uint8_t if_idx)
{
    uint8_t *buf = NULL;
    uint32_t len = 0;
    if (usbhid_get_report_buffer(uhdev, &buf, &len, USBHID_LAST) != USBHID_ERRNO_SUCCESS) {
        return -1;
    }
    if (len == 0 || len > MAX_REPORT_SIZE) {
        return -1;
    }
    hid_msg_t msg = { .if_idx = if_idx, .len = (uint16_t)len };
    memcpy(msg.data, buf, msg.len);
    if (k_msgq_put(&hid_tx_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("hid_tx_msgq full, dropping report");
        return -2;
    }
    return 0;
}

/* per-instance worker */
struct ch375_instance_args {
    const char *uart_label;
    const char *gpio_label;
    uint32_t int_pin;
    uint8_t if_idx;
};

static void dump_all_devices(void)
{
    const struct device *dev;
    size_t count = 0;
    
    LOG_INF("=== Enumerating all devices ===");
    
    #if KERNEL_VERSION_NUMBER >= ZEPHYR_VERSION(3,0,0)
        // Modern API: iterate device list
        STRUCT_SECTION_FOREACH(device, dev) {
            if (dev && dev->name) {
                LOG_INF("  [%zu] name='%s' @ %p", count++, dev->name, dev);
            }
        }
    #else
        // Older Zephyr: no portable way to enumerate all devices
        LOG_WRN("Cannot enumerate devices on Zephyr < 3.0");
    #endif
    
    LOG_INF("=== Total: %zu devices ===", count);
    (void)dev; /* silence -Wunused-variable when STRUCT_SECTION_FOREACH is empty */
}


static const struct device *try_label_list_and_log(const char *const labels[], size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (labels[i] == NULL) {
            continue;
        }
        const struct device *dev = device_get_binding(labels[i]);
        if (dev) {
            LOG_INF("device_get_binding(%s) -> %p", labels[i], dev);
            return dev;
        } else {
            LOG_DBG("device_get_binding(%s) -> NULL", labels[i]);
        }
    }
    return NULL;
}

static const struct device *find_uart2_dev(void)
{
    #if DT_NODE_HAS_STATUS(DT_ALIAS(ch375_uart_a), okay)
        const struct device *dev = DEVICE_DT_GET(DT_ALIAS(ch375_uart_a));
        if (device_is_ready(dev)) {
            LOG_INF("UART2 via DT alias: %s @ %p", dev->name, dev);
            return dev;
        }
    #endif
    
    // Fallback to device_get_binding for older Zephyr
    const char *labels[] = {"USART_2", "UART_2", "usart2", "usart@40004400", NULL};
    return try_label_list_and_log(labels, ARRAY_SIZE(labels));
}

static const struct device *find_uart3_dev(void)
{
    #if DT_NODE_HAS_STATUS(DT_ALIAS(ch375_uart_b), okay)
        const struct device *dev = DEVICE_DT_GET(DT_ALIAS(ch375_uart_b));
        if (device_is_ready(dev)) {
            LOG_INF("UART3 via DT alias: %s @ %p", dev->name, dev);
            return dev;
        }
    #endif
    
    const char *labels[] = {"USART_3", "UART_3", "usart3", "usart@40004800", NULL};
    return try_label_list_and_log(labels, ARRAY_SIZE(labels));
}

static const struct device *find_gpioe_dev(void)
{
    #if DT_NODE_HAS_STATUS(DT_ALIAS(ch375_gpio), okay)
        const struct device *dev = DEVICE_DT_GET(DT_ALIAS(ch375_gpio));
        if (device_is_ready(dev)) {
            LOG_INF("GPIOE via DT alias: %s @ %p", dev->name, dev);
            return dev;
        }
    #endif
    
    const char *labels[] = {"GPIOE", "gpioe", "gpio@40021000", NULL};
    return try_label_list_and_log(labels, ARRAY_SIZE(labels));
}

static void ch375_instance_worker(void *p1, void *p2, void *p3)
{
    struct ch375_instance_args *args = (struct ch375_instance_args *)p1;
    CH375_Context_t *ctx = NULL;
    USB_Device_t udev;
    USBHID_Device_t uhdev;
    HIDMouse_t mouse;
    HIDKeyboard_t kb;
    int ret;
    uint8_t version;
    uint8_t out_if_idx = 0;

    if (!args) {
        LOG_ERR("no args");
        return;
    }
    
    LOG_INF("CH375 worker %u: starting", args->if_idx);
    k_msleep(100 * args->if_idx);  // Stagger startup
    
    const struct device *uart_dev = (args->if_idx == 0) ? find_uart2_dev() : find_uart3_dev();
    const struct device *gpio_dev = find_gpioe_dev();

    if (!uart_dev || !gpio_dev) {
        LOG_ERR("Device binding failed for chan %u", args->if_idx);
        return;
    }

    const char *uart_label = uart_dev->name;
    const char *gpio_label = gpio_dev->name;

    LOG_INF("Chan %u: Opening CH375 adapter (uart=%s gpio=%s pin=%u)",
            args->if_idx, uart_label, gpio_label, args->int_pin);

    ret = ch375_zephyr_open(uart_label, gpio_label, args->int_pin, &ctx);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("ch375_zephyr_open=%d", ret);
        return;
    }

    LOG_INF("Chan %u: CH375 adapter opened", args->if_idx);

    ret = ch375_getVersion(ctx, &version);
    if (ret == CH375_SUCCESS) {
        LOG_INF("Chan %u: CH375 version 0x%02X", args->if_idx, version);
    } else {
        LOG_ERR("Chan %u: ch375_getVersion failed: %d", args->if_idx, ret);
    }

    ret = ch375_checkExist(ctx);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Chan %u: ch375_checkExist failed: %d", args->if_idx, ret);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: CH375 communication verified", args->if_idx);

    /* Initialize CH375 USB host mode */
    if (ch375_hostInit(ctx, 9600) != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("Chan %u: ch375_hostInit failed", args->if_idx);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: CH375 host mode initialized", args->if_idx);

    /* Wait for device connection */
    LOG_INF("Chan %u: Waiting for device connection...", args->if_idx);
    if (ch375_hostWaitDeviceConnect(ctx, 5000) != CH375_HST_ERRNO_SUCCESS) {
        LOG_WRN("Chan %u: No device connected (timeout)", args->if_idx);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: Device connected!", args->if_idx);

    /* Open USB device */
    memset(&udev, 0, sizeof(udev));
    if (ch375_hostUdevOpen(ctx, &udev) != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("Chan %u: ch375_hostUdevOpen failed", args->if_idx);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: USB device opened (VID=%04X PID=%04X)", args->if_idx, udev.vid, udev.pid);

    /* Open HID device */
    memset(&uhdev, 0, sizeof(uhdev));
    if (usbhid_open(&udev, 0, &uhdev) != USBHID_ERRNO_SUCCESS) {
        LOG_ERR("Chan %u: usbhid_open failed", args->if_idx);
        ch375_hostUdevClose(&udev);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: HID device opened (type=%u)", args->if_idx, uhdev.hid_type);

    /* Choose output composite interface index based on type */
    if (uhdev.hid_type == USBHID_TYPE_MOUSE) {
        out_if_idx = 0; /* mouse -> composite interface 0 */
    } else if (uhdev.hid_type == USBHID_TYPE_KEYBOARD) {
        out_if_idx = 1; /* keyboard -> composite interface 1 */
    } else {
        LOG_ERR("Chan %u: Unsupported HID type %u", args->if_idx, uhdev.hid_type);
        usbhid_close(&uhdev);
        ch375_hostUdevClose(&udev);
        ch375_zephyr_close(ctx);
        return;
    }

    /* Enter report read loop until device disconnects */
    if (uhdev.hid_type == USBHID_TYPE_MOUSE) {
        HIDMouse_t mouse_local;
        if (hid_mouse_open(&uhdev, &mouse_local) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("Chan %u: hid_mouse_open failed", args->if_idx);
            goto cleanup;
        }
        LOG_INF("Chan %u: Mouse opened, entering report loop", args->if_idx);

        while (1) {
            ret = hid_mouse_fetch_report(&mouse_local);
            if (ret != USBHID_ERRNO_SUCCESS) {
                if (ret == USBHID_ERRNO_NO_DEV) {
                    LOG_ERR("Chan %u: Device disconnected", args->if_idx);
                    break;
                }
                k_msleep(10);
                continue;
            }
            if (enqueue_report_from_usbhid(&uhdev, out_if_idx) != 0) {
                LOG_WRN("Chan %u: Failed to enqueue mouse report", args->if_idx);
            }
        }
        hid_mouse_close(&mouse_local);
    } else {
        HIDKeyboard_t kb_local;
        if (hid_keyboard_open(&uhdev, &kb_local) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("Chan %u: hid_keyboard_open failed", args->if_idx);
            goto cleanup;
        }
        LOG_INF("Chan %u: Keyboard opened, entering report loop", args->if_idx);

        while (1) {
            ret = hid_keyboard_fetch_report(&kb_local);
            if (ret != USBHID_ERRNO_SUCCESS) {
                if (ret == USBHID_ERRNO_NO_DEV) {
                    LOG_ERR("Chan %u: Device disconnected", args->if_idx);
                    break;
                }
                k_msleep(10);
                continue;
            }
            if (enqueue_report_from_usbhid(&uhdev, out_if_idx) != 0) {
                LOG_WRN("Chan %u: Failed to enqueue keyboard report", args->if_idx);
            }
        }
        hid_keyboard_close(&kb_local);
    }

cleanup:
    LOG_INF("Chan %u: Worker exiting", args->if_idx);
    usbhid_close(&uhdev);
    ch375_hostUdevClose(&udev);
    ch375_zephyr_close(ctx);
}


/* USB TX worker */
static void usb_tx_worker(void *p1, void *p2, void *p3)
{
    hid_msg_t msg;
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("USB TX worker started");
    for (;;) {
        if (k_msgq_get(&hid_tx_msgq, &msg, K_FOREVER) != 0) {
            continue;
        }

        LOG_DBG("usb_tx_worker: dequeued report if=%u len=%u", msg.if_idx, msg.len);

        /* Wait until USB device stack initialized and configured (with bounded retry) */
        int wait_tries = 0;
        while ((hUsbDeviceFS.pData == NULL || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) && wait_tries < 50) {
            /* small wait to allow USB stack to finish setup */
            k_msleep(10);
            wait_tries++;
        }
        if (hUsbDeviceFS.pData == NULL || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
            LOG_WRN("USB not configured after wait, requeueing report if=%u len=%u", msg.if_idx, msg.len);
            /* try to requeue once with short timeout; if fails, drop (to avoid blocking forever) */
            if (k_msgq_put(&hid_tx_msgq, &msg, K_MSEC(50)) != 0) {
                LOG_ERR("hid_tx_msgq requeue failed, dropping report");
            }
            continue;
        }

        /* For extra safety, log composite HID state before transmit */
        LOG_DBG("composite_hid: interface_cnt=%u ep1=0x%02X ep2=0x%02X",
                (unsigned)usbd_composite_hid.interface_cnt,
                usbd_composite_hid.ep_addr[0], usbd_composite_hid.ep_addr[1]);

        int rc = composite_hid_send_report(msg.if_idx, msg.data, msg.len);
        if (rc != 0) {
            LOG_ERR("composite_hid_send_report failed (rc=%d) for if=%u", rc, msg.if_idx);
            /* small backoff to avoid spamming if the host is unhappy */
            k_msleep(5);
        } else {
            LOG_DBG("Report sent (if=%u len=%u)", msg.if_idx, msg.len);
        }
    }
}

/* Instances arguments */
static struct ch375_instance_args inst_a = {
    .uart_label = CH375_A_UART_LABEL,
    .gpio_label = CH375_A_GPIO_LABEL,
    .int_pin = CH375_A_INT_PIN,
    .if_idx = 0
};
static struct ch375_instance_args inst_b = {
    .uart_label = CH375_B_UART_LABEL,
    .gpio_label = CH375_B_GPIO_LABEL,
    .int_pin = CH375_B_INT_PIN,
    .if_idx = 1
};

K_THREAD_DEFINE(hid_bridge_init_id, 16384, hid_bridge_start, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(usb_tx_thread_id, 12288, usb_tx_worker, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ch375_a_thread_id, 16384, ch375_instance_worker, &inst_a, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(ch375_b_thread_id, 16384, ch375_instance_worker, &inst_b, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(stack_monitor_id, 2048, stack_monitor_thread, NULL, NULL, NULL, 10, 0, 0);



/* hid_bridge_start uses MX_USB_DEVICE_Init() call */
void hid_bridge_start(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    LOG_INF("Init USB Device");
    LOG_INF("Register composite HID interfaces");

    if (USBD_COMPOSITE_HID_InterfaceRegister(0, NULL,
            (uint8_t *)composite_mouse_report_desc,
            sizeof(composite_mouse_report_desc),
            8, 10) != (uint8_t)USBD_OK) {
        LOG_ERR("Failed to register mouse interface");
    }

    if (USBD_COMPOSITE_HID_InterfaceRegister(1, NULL,
            (uint8_t *)composite_keyboard_report_desc,
            sizeof(composite_keyboard_report_desc),
            8, 10) != (uint8_t)USBD_OK) {
        LOG_ERR("Failed to register keyboard interface");
    }

    /* initialise composite descriptors internal state */
    if (USBD_COMPOSITE_HID_Init() != (uint8_t)USBD_OK) {
        LOG_ERR("USBD_COMPOSITE_HID_Init failed");
    }

    /* initialise composite descriptors internal state */
    if (USBD_COMPOSITE_HID_Init() != (uint8_t)USBD_OK) {
        LOG_ERR("USBD_COMPOSITE_HID_Init failed");
    } else {
        LOG_INF("composite_hid: interface_cnt=%u", usbd_composite_hid.interface_cnt);
        LOG_INF("composite_hid: ep_addr[0]=0x%02X ep_addr[1]=0x%02X",
                usbd_composite_hid.ep_addr[0], usbd_composite_hid.ep_addr[1]);
        LOG_INF("composite_hid: report_len[0]=%u report_len[1]=%u",
                usbd_composite_hid.report_desc_len[0], usbd_composite_hid.report_desc_len[1]);
    }

    /* now start the Cube USBD stack */
    MX_USB_DEVICE_Init();
}

static void stack_monitor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    
    while (1) {
        k_msleep(5000);  // Check every 5 seconds
        
        LOG_INF("=== Stack Usage Report ===");
        
        #ifdef CONFIG_THREAD_STACK_INFO
        size_t unused;
        
        if (k_thread_stack_space_get(&hid_bridge_init_id, &unused) == 0) {
            LOG_INF("  hid_bridge_init: %zu bytes unused", unused);
        }
        if (k_thread_stack_space_get(&usb_tx_thread_id, &unused) == 0) {
            LOG_INF("  usb_tx: %zu bytes unused", unused);
        }
        if (k_thread_stack_space_get(&ch375_a_thread_id, &unused) == 0) {
            LOG_INF("  ch375_a: %zu bytes unused", unused);
        }
        if (k_thread_stack_space_get(&ch375_b_thread_id, &unused) == 0) {
            LOG_INF("  ch375_b: %zu bytes unused", unused);
        }
        #endif
    }
}