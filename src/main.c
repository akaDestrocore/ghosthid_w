/* Main application for USB HID Proxy on Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ch375/ch375.h"
#include "ch375/ch375_host.h"
#include "ch375/ch375_uart.h"
#include "hid/hid_mouse.h"
#include "hid/hid_keyboard.h"
#include "usb/composite_hid.h"
#include "auto_gun_press.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define CH375_MODULE_NUM 2
#define STACK_SIZE 2048
#define PRIORITY 7

typedef struct device_input {
    const char *name;
    const struct device *uart_dev;
    const struct device *gpio_dev;
    gpio_pin_t int_pin;
    
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

static device_input_t s_arr_devin[CH375_MODULE_NUM];
static struct agp_context *s_agp_ctx;
static uint8_t s_enable_gun_press;
static uint8_t s_started_gun_press;

/* Work queue for handling USB HID processing */
K_THREAD_STACK_DEFINE(hid_stack, STACK_SIZE);
struct k_work_q hid_work_q;

/* Semaphores for synchronization */
K_SEM_DEFINE(devin_ready_sem, 0, 1);

/* USB HID report descriptors (will be populated from connected devices) */
static uint8_t mouse_report_desc[256];
static uint8_t keyboard_report_desc[256];
static uint16_t mouse_report_desc_len;
static uint16_t keyboard_report_desc_len;

/* USB HID callbacks for device mode */
static void hid_int_ep_in(void)
{
    /* Handle IN endpoint interrupt */
}

static void hid_int_ep_out(uint8_t *buf, size_t len)
{
    /* Handle OUT endpoint data */
}

static int hid_get_report(struct usb_setup_packet *setup, int32_t *len, uint8_t **data)
{
    /* Handle GET_REPORT request */
    return 0;
}

static int hid_set_report(struct usb_setup_packet *setup, int32_t *len, uint8_t **data)
{
    /* Handle SET_REPORT request */
    return 0;
}

static const struct hid_ops hid_ops = {
    .int_in_ready = hid_int_ep_in,
    .int_out_ready = hid_int_ep_out,
    .get_report = hid_get_report,
    .set_report = hid_set_report,
};

/* Initialize UART with 9-bit mode for CH375 */
static int uart_init_9bit(const struct device *uart_dev, uint32_t baudrate)
{
    struct uart_config cfg = {
        .baudrate = baudrate,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .data_bits = UART_CFG_DATA_BITS_9,  /* 9-bit mode */
    };
    
    return uart_configure(uart_dev, &cfg);
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

/* Initialize device input */
static int init_device_input(device_input_t *devin, const char *uart_name,
                            const char *gpio_name, gpio_pin_t int_pin,
                            const char *dev_name, uint8_t interface_num)
{
    int ret;
    
    devin->name = dev_name;
    devin->interface_num = interface_num;
    devin->int_pin = int_pin;
    
    /* Get UART device */
    devin->uart_dev = device_get_binding(uart_name);
    if (!devin->uart_dev) {
        LOG_ERR("Cannot find UART device %s", uart_name);
        return -ENODEV;
    }
    
    /* Configure UART for 9-bit mode at default baudrate */
    ret = uart_init_9bit(devin->uart_dev, CH375_DEFAULT_BAUDRATE);
    if (ret) {
        LOG_ERR("Failed to configure UART %s: %d", uart_name, ret);
        return ret;
    }
    
    /* Get GPIO device */
    devin->gpio_dev = device_get_binding(gpio_name);
    if (!devin->gpio_dev) {
        LOG_ERR("Cannot find GPIO device %s", gpio_name);
        return -ENODEV;
    }
    
    /* Configure interrupt pin */
    ret = gpio_pin_configure(devin->gpio_dev, devin->int_pin,
                            GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure INT pin: %d", ret);
        return ret;
    }
    
    /* Create CH375 context */
    ret = ch375_open_context(&devin->ch375_ctx,
                            ch375_write_cmd,
                            ch375_write_data,
                            ch375_read_data,
                            ch375_query_int,
                            devin);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Failed to create CH375 context: %d", ret);
        return -EIO;
    }
    
    /* Initialize CH375 in host mode */
    ret = ch375_host_init(devin->ch375_ctx, CH375_WORK_BAUDRATE);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Failed to initialize CH375 host: %d", ret);
        return -EIO;
    }
    
    /* Reconfigure UART for working baudrate */
    ret = uart_init_9bit(devin->uart_dev, CH375_WORK_BAUDRATE);
    if (ret) {
        LOG_ERR("Failed to reconfigure UART: %d", ret);
        return ret;
    }
    
    LOG_INF("Initialized %s", devin->name);
    return 0;
}

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

void main(void)
{
    int ret;
    
    LOG_INF("USB HID Proxy starting...");
    
    /* Initialize device inputs */
    ret = init_device_input(&s_arr_devin[0], "UART_2", "GPIOE", 14, "CH375A", 0);
    if (ret) {
        LOG_ERR("Failed to initialize CH375A");
        return;
    }
    
    ret = init_device_input(&s_arr_devin[1], "UART_3", "GPIOE", 15, "CH375B", 1);
    if (ret) {
        LOG_ERR("Failed to initialize CH375B");
        return;
    }
    
    /* Initialize auto gun press */
    ret = agp_open(&s_agp_ctx);
    if (ret) {
        LOG_ERR("Failed to initialize auto gun press");
        return;
    }
    
    /* Initialize work queue */
    k_work_queue_init(&hid_work_q);
    k_work_queue_start(&hid_work_q, hid_stack,
                       K_THREAD_STACK_SIZEOF(hid_stack),
                       PRIORITY, NULL);
    
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
        
        /* Wait for disconnection */
        k_sem_take(&devin_ready_sem, K_FOREVER);
    }
    
    agp_close(s_agp_ctx);
}