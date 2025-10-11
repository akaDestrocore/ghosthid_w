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
                                const char *uart_name,
                                const struct gpio_dt_spec *int_gpio,
                                const char *dev_name,
                                uint8_t interface_num)
{
    int ret;
    
    devin->name = dev_name;
    devin->interface_num = interface_num;
    devin->int_gpio = *int_gpio;
    
    /* Get UART device from device tree */
    devin->uart_dev = device_get_binding(uart_name);
    if (!devin->uart_dev) {
        LOG_ERR("Cannot find UART device %s", uart_name);
        return -ENODEV;
    }
    
    LOG_INF("Found UART device: %s", uart_name);
    
    /* Initialize CH375 hardware (includes 9-bit UART config) */
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
    
    /* Define INT GPIO specs from device tree */
    /* Use 'gpios' property instead of 'int-gpios' since we're using gpio-leds compatible */
    static const struct gpio_dt_spec ch375a_int = GPIO_DT_SPEC_GET(DT_NODELABEL(ch375a), gpios);
    static const struct gpio_dt_spec ch375b_int = GPIO_DT_SPEC_GET(DT_NODELABEL(ch375b), gpios);
    
    LOG_INF("USB HID Proxy starting...");
    
    /* Initialize device inputs using device tree */
    ret = init_device_input_dt(&s_arr_devin[0], "USART_2", &ch375a_int, "CH375A", 0);
    if (ret) {
        LOG_ERR("Failed to initialize CH375A");
        return ret;
    }
    
    ret = init_device_input_dt(&s_arr_devin[1], "USART_3", &ch375b_int, "CH375B", 1);
    if (ret) {
        LOG_ERR("Failed to initialize CH375B");
        return ret;
    }
    
    /* Initialize auto gun press */
    ret = agp_open(&s_agp_ctx);
    if (ret) {
        LOG_ERR("Failed to initialize auto gun press");
        return ret;
    }
    
    /* Initialize work queue */
    k_work_queue_init(&hid_work_q);
    k_work_queue_start(&hid_work_q, hid_stack,
                       K_THREAD_STACK_SIZEOF(hid_stack),
                       7, NULL);
    
    while (1) {
        LOG_INF("Waiting for all devices to connect...");
        
        /* Wait for all devices to connect */
        for (int i = 0; i < CH375_MODULE_NUM; i++) {
            ret = wait_and_enumerate_device(&s_arr_devin[i]);
            if (ret) {
                LOG_ERR("Failed to enumerate device %d", i);
                k_msleep(1000);
                continue;
            }
        }
        
        /* Initialize USB device mode */
        ret = init_usb_device();
        if (ret) {
            LOG_ERR("Failed to initialize USB device");
            k_msleep(1000);
            continue;
        }
        
        /* Start HID processing */
        k_work_submit_to_queue(&hid_work_q, &hid_work);
        
        /* Keep running - disconnection will be handled in work handler */
        k_sleep(K_FOREVER);
    }
    
    agp_close(s_agp_ctx);
    return 0;
}