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

/* composite wrapper */
extern int composite_hid_send_report(uint8_t interface_idx, uint8_t *report, uint16_t len);

/* USB init from usbd_conf.c */
extern void MX_USB_DEVICE_Init(void);

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
    LOG_INF("Chan %u: USB device opened (VID=%04X PID=%04X)", 
            args->if_idx, udev.vid, udev.pid);

    /* Open HID device */
    memset(&uhdev, 0, sizeof(uhdev));
    if (usbhid_open(&udev, 0, &uhdev) != USBHID_ERRNO_SUCCESS) {
        LOG_ERR("Chan %u: usbhid_open failed", args->if_idx);
        ch375_hostUdevClose(&udev);
        ch375_zephyr_close(ctx);
        return;
    }
    LOG_INF("Chan %u: HID device opened (type=%u)", args->if_idx, uhdev.hid_type);

    /* Open specific HID type */
    if (uhdev.hid_type == USBHID_TYPE_MOUSE) {
        if (hid_mouse_open(&uhdev, &mouse) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("Chan %u: hid_mouse_open failed", args->if_idx);
            usbhid_close(&uhdev);
            ch375_hostUdevClose(&udev);
            ch375_zephyr_close(ctx);
            return;
        }
        LOG_INF("Chan %u: Mouse opened, entering report loop", args->if_idx);
        
        while (1) {
            ret = hid_mouse_fetch_report(&mouse);
            if (ret != USBHID_ERRNO_SUCCESS) {
                if (ret == USBHID_ERRNO_NO_DEV) {
                    LOG_ERR("Chan %u: Device disconnected", args->if_idx);
                    break;
                }
                k_msleep(10);
                continue;
            }
            if (enqueue_report_from_usbhid(&uhdev, args->if_idx) != 0) {
                LOG_WRN("Chan %u: Failed to enqueue mouse report", args->if_idx);
            }
        }
        
    } else if (uhdev.hid_type == USBHID_TYPE_KEYBOARD) {
        if (hid_keyboard_open(&uhdev, &kb) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("Chan %u: hid_keyboard_open failed", args->if_idx);
            usbhid_close(&uhdev);
            ch375_hostUdevClose(&udev);
            ch375_zephyr_close(ctx);
            return;
        }
        LOG_INF("Chan %u: Keyboard opened, entering report loop", args->if_idx);
        
        while (1) {
            ret = hid_keyboard_fetch_report(&kb);
            if (ret != USBHID_ERRNO_SUCCESS) {
                if (ret == USBHID_ERRNO_NO_DEV) {
                    LOG_ERR("Chan %u: Device disconnected", args->if_idx);
                    break;
                }
                k_msleep(10);
                continue;
            }
            if (enqueue_report_from_usbhid(&uhdev, args->if_idx) != 0) {
                LOG_WRN("Chan %u: Failed to enqueue keyboard report", args->if_idx);
            }
        }
        
    } else {
        LOG_ERR("Chan %u: Unsupported HID type %u", args->if_idx, uhdev.hid_type);
    }

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
    while (1) {
        if (k_msgq_get(&hid_tx_msgq, &msg, K_FOREVER) != 0) {
            continue;
        }
        if (composite_hid_send_report(msg.if_idx, msg.data, msg.len) != 0) {
            LOG_ERR("composite_hid_send_report failed");
            k_msleep(5);
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

K_THREAD_DEFINE(hid_bridge_init_id, 2048, hid_bridge_start, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(usb_tx_thread_id, 4096, usb_tx_worker, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ch375_a_thread_id, 8192, ch375_instance_worker, &inst_a, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(ch375_b_thread_id, 8192, ch375_instance_worker, &inst_b, NULL, NULL, 7, 0, 0);

/* hid_bridge_start uses MX_USB_DEVICE_Init() call */
void hid_bridge_start(void *p1, void *p2, void *p3)
{
    LOG_INF("Zephyr version: %d.%d.%d", 
        KERNEL_VERSION_MAJOR, 
        KERNEL_VERSION_MINOR, 
        KERNEL_PATCHLEVEL);

    #ifdef DEVICE_DT_GET
        LOG_INF("Using modern device tree API (DEVICE_DT_GET available)");
    #else
        LOG_INF("Using legacy device_get_binding API");
    #endif


    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    LOG_INF("Init USB Device");
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

K_THREAD_DEFINE(stack_monitor_id, 1024, stack_monitor_thread, NULL, NULL, NULL, 10, 0, 0);