/* Main application - Zephyr style with proper device tree usage */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>

#include "ch375/ch375.h"
#include "ch375/ch375_host.h"
#include "ch375/ch375_uart.h"
#include "hid/hid_mouse.h"
#include "hid/hid_keyboard.h"
#include "usb/composite_hid.h"
#include "auto_gun_press.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define CH375_MODULE_NUM 2

/* Device context structure */
typedef struct {
    const char *name;
    const struct device *uart_dev;
    struct gpio_dt_spec int_gpio;
    
    struct ch375_context *ch375_ctx;
    struct usb_device usbdev;
    struct usbhid_device hiddev;
    union {
        struct hid_mouse mouse;
        struct hid_keyboard keyboard;
    };
    
    uint8_t is_connected;
    uint8_t interface_num;
} device_input_t;

/* Global state */
static device_input_t s_arr_devin[CH375_MODULE_NUM];
static struct agp_context *s_agp_ctx;
static uint8_t s_enable_gun_press;
static uint8_t s_started_gun_press;

/* Thread stacks */
K_THREAD_STACK_DEFINE(hid_stack, 2048);
struct k_work_q hid_work_q;

/* Device initialization using device tree */
static int init_device_input_dt(device_input_t *devin, 
                                const char *uart_label,  /* Keep for logging */
                                const struct device *uart_dev,  /* Pass device directly */
                                const struct gpio_dt_spec *int_gpio,
                                const char *dev_name,
                                uint8_t interface_num)
{
    int ret;
    
    devin->name = dev_name;
    devin->interface_num = interface_num;
    devin->int_gpio = *int_gpio;
    devin->uart_dev = uart_dev;  /* Use passed device */
    
    if (!device_is_ready(devin->uart_dev)) {
        LOG_ERR("UART device %s not ready", uart_label);
        return -ENODEV;
    }
    
    LOG_INF("Found UART device: %s", uart_label);
    
    /* Rest remains the same */
    ret = ch375_hw_init(devin->name,
                       devin->uart_dev,
                       devin->int_gpio,
                       CH375_DEFAULT_BAUDRATE,
                       &devin->ch375_ctx);
    if (ret < 0) {
        LOG_ERR("Failed to init CH375 hardware: %d", ret);
        return ret;
    }
    
    /* Initialize CH375 in host mode */
    ret = ch375_host_init(devin->ch375_ctx, CH375_WORK_BAUDRATE);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Failed to initialize CH375 host: %d", ret);
        return -EIO;
    }
    
    /* Reconfigure UART for working baudrate */
    ret = ch375_hw_set_baudrate(devin->ch375_ctx, CH375_WORK_BAUDRATE);
    if (ret < 0) {
        LOG_ERR("Failed to set working baudrate: %d", ret);
        return ret;
    }
    
    LOG_INF("Initialized %s", devin->name);
    return 0;
}

/* Process mouse HID reports */
static int handle_mouse(struct hid_mouse *mouse, uint8_t interface_num)
{
    struct usbhid_device *hiddev = mouse->hid_dev;
    uint8_t *report_buf = NULL;
    int ret;
    uint32_t button_value;
    uint8_t fetch_success = 0;
    uint8_t need_send = 0;
    
    ret = hid_mouse_fetch_report(mouse);
    if (ret == USBHID_SUCCESS) {
        fetch_success = 1;
        need_send = 1;
    }
    
    usbhid_get_report_buffer(hiddev, &report_buf, NULL, USBHID_NOW);
    
    /* Check left mouse button for auto gun press */
    hid_mouse_get_button(mouse, HID_MOUSE_BUTTON_LEFT, &button_value, USBHID_NOW);
    if (button_value) {
        if (!s_started_gun_press) {
            s_started_gun_press = 1;
            agp_restart(s_agp_ctx);
        }
    } else {
        s_started_gun_press = 0;
    }
    
    /* Apply auto gun press if enabled */
    if (s_enable_gun_press && s_started_gun_press) {
        struct agp_data data;
        ret = agp_get_data(s_agp_ctx, &data);
        if (ret == 0) {
            int32_t value;
            
            /* Add compensation to existing mouse movement */
            if (fetch_success) {
                hid_mouse_get_orientation(mouse, HID_MOUSE_AXIS_X, &value, USBHID_NOW);
                data.x += value;
                hid_mouse_get_orientation(mouse, HID_MOUSE_AXIS_Y, &value, USBHID_NOW);
                data.y += value;
            }
            
            hid_mouse_set_orientation(mouse, HID_MOUSE_AXIS_X, data.x, USBHID_NOW);
            hid_mouse_set_orientation(mouse, HID_MOUSE_AXIS_Y, data.y, USBHID_NOW);
            need_send = 1;
        }
    }
    
    if (need_send) {
        /* Send HID report via USB device */
        hid_device_send_report(interface_num, report_buf, hiddev->report_length);
    }
    
    return 0;
}

/* Process keyboard HID reports */
static int handle_keyboard(struct hid_keyboard *keyboard, uint8_t interface_num)
{
    struct usbhid_device *hiddev = keyboard->hid_dev;
    uint8_t *report_buf = NULL;
    int ret;
    uint32_t value;
    
    ret = hid_keyboard_fetch_report(keyboard);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    usbhid_get_report_buffer(hiddev, &report_buf, NULL, USBHID_NOW);
    
    /* Check for control keys */
    hid_keyboard_get_key(keyboard, HID_KBD_NUMBER(1), &value, USBHID_NOW);
    if (value) {
        s_enable_gun_press = 1;
        LOG_INF("Auto gun press enabled");
    }
    
    hid_keyboard_get_key(keyboard, HID_KBD_NUMBER(2), &value, USBHID_NOW);
    if (value) {
        s_enable_gun_press = 0;
        LOG_INF("Auto gun press disabled");
    }
    
    /* Check for weapon selection */
    hid_keyboard_get_key(keyboard, HID_KBD_LETTER('k'), &value, USBHID_NOW);
    if (value) {
        agp_set_collect(s_agp_ctx, AGP_COLLECT_IDX_AK47);
        LOG_INF("Selected AK47 profile");
    }
    
    hid_keyboard_get_key(keyboard, HID_KBD_LETTER('m'), &value, USBHID_NOW);
    if (value) {
        agp_set_collect(s_agp_ctx, AGP_COLLECT_IDX_M4A4);
        LOG_INF("Selected M4A4 profile");
    }
    
    /* Coefficient adjustment */
    hid_keyboard_get_key(keyboard, HID_KBD_EQUAL, &value, USBHID_NOW);
    if (value) {
        agp_coefficient_change(s_agp_ctx, 1);
    }
    
    hid_keyboard_get_key(keyboard, HID_KBD_MINUS, &value, USBHID_NOW);
    if (value) {
        agp_coefficient_change(s_agp_ctx, 0);
    }
    
    /* Send report */
    hid_device_send_report(interface_num, report_buf, hiddev->report_length);
    
    return 0;
}

/* HID processing work handler */
static void hid_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int ret;
    
    while (1) {
        for (int i = 0; i < CH375_MODULE_NUM; i++) {
            device_input_t *devin = &s_arr_devin[i];
            
            if (!devin->is_connected) {
                continue;
            }
            
            if (devin->hiddev.hid_type == USBHID_TYPE_MOUSE) {
                ret = handle_mouse(&devin->mouse, devin->interface_num);
            } else if (devin->hiddev.hid_type == USBHID_TYPE_KEYBOARD) {
                ret = handle_keyboard(&devin->keyboard, devin->interface_num);
            }
            
            if (ret == USBHID_NO_DEV) {
                LOG_ERR("%s: Device disconnected", devin->name);
                devin->is_connected = 0;
            }
        }
        
        k_msleep(1);  /* 1ms polling interval */
    }
}

K_WORK_DEFINE(hid_work, hid_work_handler);

/* Wait for device connection and enumerate */
static int wait_and_enumerate_device(device_input_t *devin)
{
    int ret;
    
    LOG_INF("%s: Waiting for device connection...", devin->name);
    
    ret = ch375_host_wait_device_connect(devin->ch375_ctx, 10000);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("%s: Device connection timeout", devin->name);
        return -ETIMEDOUT;
    }
    
    LOG_INF("%s: Device connected, enumerating...", devin->name);
    
    /* Open USB device */
    ret = ch375_host_udev_open(devin->ch375_ctx, &devin->usbdev);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("%s: Failed to open USB device: %d", devin->name, ret);
        return -EIO;
    }
    
    LOG_INF("%s: USB device opened (VID:%04X PID:%04X)",
           devin->name, devin->usbdev.vid, devin->usbdev.pid);
    
    /* Open HID device */
    ret = usbhid_open(&devin->usbdev, 0, &devin->hiddev);
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("%s: Failed to open HID device: %d", devin->name, ret);
        return -EIO;
    }
    
    LOG_INF("%s: HID device opened (type: %s)",
           devin->name,
           devin->hiddev.hid_type == USBHID_TYPE_MOUSE ? "Mouse" : "Keyboard");
    
    /* Initialize HID class driver */
    if (devin->hiddev.hid_type == USBHID_TYPE_MOUSE) {
        ret = hid_mouse_open(&devin->hiddev, &devin->mouse);
    } else {
        ret = hid_keyboard_open(&devin->hiddev, &devin->keyboard);
    }
    
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("%s: Failed to open HID class device: %d", devin->name, ret);
        return -EIO;
    }
    
    devin->is_connected = 1;
    return 0;
}

/* USB device initialization */
static int init_usb_device(void)
{
    int ret;
    
    /* Register composite HID interfaces */
    for (int i = 0; i < CH375_MODULE_NUM; i++) {
        device_input_t *devin = &s_arr_devin[i];
        
        if (!devin->is_connected) {
            continue;
        }
        
        /* Register HID interface */
        ret = composite_hid_register_interface(
            i,
            devin->hiddev.raw_hid_report_desc,
            devin->hiddev.raw_hid_report_desc_len,
            8,  /* max packet size */
            1   /* interval */
        );
        
        if (ret) {
            LOG_ERR("Failed to register HID interface %d", i);
            return ret;
        }
    }
    
    /* Initialize composite HID */
    ret = composite_hid_init();
    if (ret) {
        LOG_ERR("Failed to initialize composite HID");
        return ret;
    }
    
    /* Enable USB device */
    ret = usb_enable(NULL);
    if (ret) {
        LOG_ERR("Failed to enable USB device");
        return ret;
    }
    
    LOG_INF("USB device initialized");
    return 0;
}

int main(void)
{
    int ret;
    
    /* Define INT GPIO specs directly - PC13 and PC14 */
    static const struct gpio_dt_spec ch375a_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };
    
    static const struct gpio_dt_spec ch375b_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 14,
        .dt_flags = GPIO_ACTIVE_LOW
    };
    
    LOG_INF("=================================");
    LOG_INF("USB HID Proxy Starting...");
    LOG_INF("=================================");
    
    /* Test GPIO access */
    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    if (!device_is_ready(gpioc)) {
        LOG_ERR("GPIO C not ready!");
        return -1;
    }
    LOG_INF("GPIO C is ready");
    
    /* Get UART devices using DT macros - THIS IS THE FIX */
    const struct device *uart2 = DEVICE_DT_GET(DT_NODELABEL(usart2));
    if (!device_is_ready(uart2)) {
        LOG_ERR("USART2 not ready");
        return -1;
    }
    LOG_INF("USART2 ready");
    
    const struct device *uart3 = DEVICE_DT_GET(DT_NODELABEL(usart3));
    if (!device_is_ready(uart3)) {
        LOG_ERR("USART3 not ready");
        return -1;
    }
    LOG_INF("USART3 ready");
    
    const struct device *uart4 = DEVICE_DT_GET(DT_NODELABEL(uart4));
    if (!device_is_ready(uart4)) {
        LOG_ERR("UART4 not ready");
        return -1;
    }
    LOG_INF("UART4 ready");
    
    LOG_INF("=================================");
    LOG_INF("Basic tests complete");
    LOG_INF("=================================");

    /* Initialize device inputs */
ret = init_device_input_dt(&s_arr_devin[0], "USART2", uart2, &ch375a_int, "CH375A", 0);
if (ret) {
    LOG_ERR("Failed to initialize CH375A");
    return ret;
}

ret = init_device_input_dt(&s_arr_devin[1], "USART3", uart3, &ch375b_int, "CH375B", 1);
if (ret) {
    LOG_ERR("Failed to initialize CH375B");
    return ret;
}

    
    /* Simple loop */
    while (1) {
        LOG_INF("Heartbeat...");
        k_msleep(2000);
    }
    
    return 0;
}