#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
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
    const char *labels[] = {
        "USART_2",
        "UART_2",
        "usart2",
        NULL
    };
    return try_label_list_and_log(labels, ARRAY_SIZE(labels));
}

static const struct device *find_uart3_dev(void)
{
    const char *labels[] = {
        "USART_3",
        "UART_3",
        "usart3",
        NULL
    };
    return try_label_list_and_log(labels, ARRAY_SIZE(labels));
}

static const struct device *find_gpioe_dev(void)
{
    const char *labels[] = {
        "GPIOE",
        "gpioe",
        NULL
    };
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

    if (!args) {
        LOG_ERR("no args");
        return;
    }
    LOG_INF("CH375 worker start for UART=%s GPIO=%s INT=%u -> if=%u",
            args->uart_label, args->gpio_label, args->int_pin, args->if_idx);

    /*********************************************************************************************** */
    /* resolve device handles (try DT label then fallbacks) */
    const struct device *uart_dev = (args->if_idx == 0) ? find_uart2_dev() : find_uart3_dev();
    const struct device *gpio_dev = find_gpioe_dev();

    if (!uart_dev || !gpio_dev) {
        LOG_ERR("device binding checks failed for chan %u (uart=%p gpio=%p)",
                args->if_idx, uart_dev, gpio_dev);
        return;
    }

    /* device->name is a string usable by legacy APIs that expect a label */
    const char *resolved_uart_label = uart_dev->name;
    const char *resolved_gpio_label = gpio_dev->name;

    LOG_INF("Resolved labels for chan %u: uart=%s gpio=%s",
            args->if_idx, resolved_uart_label, resolved_gpio_label);

    /* Use the resolved labels only (drop the old args->uart_label call) */
    ret = ch375_zephyr_open(resolved_uart_label, resolved_gpio_label, args->int_pin, &ctx);
    /*********************************************************************************************** */
    if (ret != CH375_SUCCESS) {
        LOG_ERR("ch375_zephyr_open=%d", ret);
        return;
    }

    if (ch375_hostInit(ctx, 9600) != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("ch375_hostInit failed");
        ch375_zephyr_close(ctx);
        return;
    }

    if (ch375_hostWaitDeviceConnect(ctx, 5000) != CH375_HST_ERRNO_SUCCESS) {
        LOG_WRN("no device connected to CH375 (timeout)");
        ch375_zephyr_close(ctx);
        return;
    }

    memset(&udev, 0, sizeof(udev));
    if (ch375_hostUdevOpen(ctx, &udev) != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("ch375_hostUdevOpen failed");
        ch375_zephyr_close(ctx);
        return;
    }

    memset(&uhdev, 0, sizeof(uhdev));
    if (usbhid_open(&udev, 0, &uhdev) != USBHID_ERRNO_SUCCESS) {
        LOG_ERR("usbhid_open failed");
        ch375_hostUdevClose(&udev);
        ch375_zephyr_close(ctx);
        return;
    }

    if (uhdev.hid_type == USBHID_TYPE_MOUSE) {
        if (hid_mouse_open(&uhdev, &mouse) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("hid_mouse_open failed");
            usbhid_close(&uhdev);
            ch375_zephyr_close(ctx);
            return;
        }
        while (1) {
            ret = hid_mouse_fetch_report(&mouse);
            if (ret != USBHID_ERRNO_SUCCESS) {
                k_msleep(100);
                continue;
            }
            enqueue_report_from_usbhid(&uhdev, args->if_idx);
        }
    } else if (uhdev.hid_type == USBHID_TYPE_KEYBOARD) {
        if (hid_keyboard_open(&uhdev, &kb) != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("hid_keyboard_open failed");
            usbhid_close(&uhdev);
            ch375_zephyr_close(ctx);
            return;
        }
        while (1) {
            ret = hid_keyboard_fetch_report(&kb);
            if (ret != USBHID_ERRNO_SUCCESS) {
                k_msleep(100);
                continue;
            }
            enqueue_report_from_usbhid(&uhdev, args->if_idx);
        }
    } else {
        LOG_ERR("unsupported HID type %u", uhdev.hid_type);
    }
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

/* Auto-start thread definitions (K_THREAD_DEFINE creates stack+TCB statically) */
K_THREAD_DEFINE(hid_bridge_init_id, 1024, hid_bridge_start, NULL, NULL, NULL, 6, 0, 0);

/* usb tx */
K_THREAD_DEFINE(usb_tx_thread_id, 2048, usb_tx_worker, NULL, NULL, NULL, 5, 0, 0);

/* per-ch375 threads */
K_THREAD_DEFINE(ch375_a_thread_id, 4096, ch375_instance_worker, &inst_a, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(ch375_b_thread_id, 4096, ch375_instance_worker, &inst_b, NULL, NULL, 7, 0, 0);

/* hid_bridge_start uses MX_USB_DEVICE_Init() call */
void hid_bridge_start(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    LOG_INF("Init USB Device");
    MX_USB_DEVICE_Init();
}
