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
#include "hid/async_report.h"
#include "usb/composite_hid.h"
#include "auto_gun_press.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/************************************************* */
uint8_t mouse_report[4] = {0x00, 5, 0, 0}; // buttons, x, y, wheel
/************************************************* */

#define CH375_MODULE_NUM 2

void composite_hid_set_usb_configured(bool configured);

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

/* Device initialization using device tree */
static int init_device_input_dt(device_input_t *devin, 
                                const char *uart_label,
                                const struct device *uart_dev,
                                const struct gpio_dt_spec *int_gpio,
                                const char *dev_name,
                                uint8_t interface_num)
{
    int ret;
    
    devin->name = dev_name;
    devin->interface_num = interface_num;
    devin->int_gpio = *int_gpio;
    devin->uart_dev = uart_dev;
    
    if (!device_is_ready(devin->uart_dev)) {
        LOG_ERR("UART device %s not ready", uart_label);
        return -ENODEV;
    }
    
    LOG_INF("Found UART device: %s", uart_label);
    
    /* Initialize CH375 hardware */
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

static void send_test_mouse_move(uint8_t iface)
{
    /* Typical boot-mouse: [buttons, dx, dy] (signed 8-bit deltas) */
    const int8_t dx = 10; /* move right */
    const int8_t dy = 0;
    uint8_t rpt[3];
    rpt[0] = 0x00;            /* buttons */
    rpt[1] = (uint8_t)dx;     /* X */
    rpt[2] = (uint8_t)dy;     /* Y */

    int rc = hid_device_send_report(iface, rpt, sizeof(rpt));
    LOG_INF("test mouse send rc=%d", rc);
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
    
    k_mutex_lock(&mouse->hid_dev->report_lock, K_FOREVER);
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
        int rc = composite_hid_send_report_async(interface_num, report_buf, hiddev->report_length);
        if (rc == -EAGAIN) {
            /* USB not ready yet; skip this report */
            LOG_DBG("USB not ready for send (iface %d), skipping", interface_num);
        } else if (rc != 0) {
            LOG_ERR("hid_device_send_report failed: %d", rc);
        }
    }
    
    k_mutex_unlock(&mouse->hid_dev->report_lock);
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
    
    k_mutex_lock(&keyboard->hid_dev->report_lock, K_FOREVER);
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
    composite_hid_send_report_async(interface_num, report_buf, hiddev->report_length);
    
    k_mutex_unlock(&keyboard->hid_dev->report_lock);
    return 0;
}

/* Open device input (USB HID host side) */
static int open_device_in(device_input_t *devin)
{
    struct usb_device *udev = &devin->usbdev;
    struct usbhid_device *hiddev = &devin->hiddev;
    struct hid_mouse *mouse = &devin->mouse;
    struct hid_keyboard *keyboard = &devin->keyboard;
    int ret;
    
    LOG_INF("%s: Opening USB device...", devin->name);
    
    ret = ch375_host_udev_open(devin->ch375_ctx, udev);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("%s: Failed to open USB device: %d", devin->name, ret);
        return -1;
    }
    
    LOG_INF("%s: USB device opened (VID:PID = %04X:%04X)",
           devin->name, udev->vid, udev->pid);
    
    /* Open USBHID device (interface 0) */
    ret = usbhid_open(udev, 0, hiddev);
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("%s: Failed to open USBHID: %d", devin->name, ret);
        ch375_host_udev_close(udev);
        return -1;
    }
    
    LOG_INF("%s: USBHID opened (type: %s)",
           devin->name,
           hiddev->hid_type == USBHID_TYPE_MOUSE ? "Mouse" :
           hiddev->hid_type == USBHID_TYPE_KEYBOARD ? "Keyboard" : "Unknown");
    
    /* Open HID class device */
    if (hiddev->hid_type == USBHID_TYPE_MOUSE) {
        ret = hid_mouse_open(hiddev, mouse);
        if (ret != USBHID_SUCCESS) {
            LOG_ERR("%s: Failed to open mouse: %d", devin->name, ret);
            goto cleanup;
        }
        LOG_INF("%s: Mouse opened", devin->name);
    } else if (hiddev->hid_type == USBHID_TYPE_KEYBOARD) {
        ret = hid_keyboard_open(hiddev, keyboard);
        if (ret != USBHID_SUCCESS) {
            LOG_ERR("%s: Failed to open keyboard: %d", devin->name, ret);
            goto cleanup;
        }
        LOG_INF("%s: Keyboard opened", devin->name);
    } else {
        LOG_ERR("%s: Unsupported HID type: %d", devin->name, hiddev->hid_type);
        goto cleanup;
    }
    
    return 0;

cleanup:
    usbhid_close(hiddev);
    ch375_host_udev_close(udev);
    return -1;
}

/* Wait for all devices to connect */
static void wait_all_device_connect(void)
{
    uint8_t all_connected = 0;
    int ret;
    
    while (!all_connected) {
        all_connected = 1;
        
        for (int i = 0; i < CH375_MODULE_NUM; i++) {
            device_input_t *devin = &s_arr_devin[i];
            
            if (devin->is_connected) {
                continue;
            }
            
            ret = ch375_host_wait_device_connect(devin->ch375_ctx, 500);
            if (ret == CH375_HOST_SUCCESS) {
                LOG_INF("%s: Device connected", devin->name);
                devin->is_connected = 0;  /* Not fully enumerated yet */
            } else if (ret == CH375_HOST_ERROR) {
                LOG_ERR("%s: Error waiting for device", devin->name);
                all_connected = 0;
            } else {
                /* Timeout - keep waiting */
                all_connected = 0;
            }
        }
        
        if (!all_connected) {
            k_msleep(100);
        }
    }
}

/* Open all input devices */
static int open_all_device_in(void)
{
    int ret;
    
    for (int i = 0; i < CH375_MODULE_NUM; i++) {
        ret = open_device_in(&s_arr_devin[i]);
        if (ret < 0) {
            LOG_ERR("%s: Failed to enumerate", s_arr_devin[i].name);
            return ret;
        }
        s_arr_devin[i].is_connected = 1;
    }
    
    return 0;
}

/* Open device output (USB composite HID) */
static void open_device_out(void)
{
    uint8_t status;
    
    for (int i = 0; i < CH375_MODULE_NUM; i++) {
        device_input_t *devin = &s_arr_devin[i];
        struct usbhid_device *hiddev = &devin->hiddev;
        
        if (!devin->is_connected) {
            continue;
        }
        
        status = composite_hid_register_interface(i,
            hiddev->raw_hid_report_desc,
            hiddev->raw_hid_report_desc_len,
            8, 1);
            
        if (status != 0) {
            LOG_ERR("Failed to register HID interface %d", i);
        }
        LOG_INF("Register iface %d: report_desc_len=%d, max_packet=%d",
        i, hiddev->raw_hid_report_desc_len, 8);
    }
    
    status = composite_hid_init();
    if (status != 0) {
        LOG_ERR("Failed to initialize composite HID");
    }
}

/* Main HID processing loop */
static void loop_handle_devices(void)
{
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
            } else {
                continue;
            }
            
            if (ret == USBHID_NO_DEV) {
                LOG_ERR("%s: Device disconnected", devin->name);
                return;  /* Exit loop to restart */
            }
        }
        
        k_msleep(1);  /* 1ms polling interval */
    }
}

/* Close all devices */
static void close_all_devices(void)
{
    for (int i = 0; i < CH375_MODULE_NUM; i++) {
        device_input_t *devin = &s_arr_devin[i];
        
        if (devin->hiddev.hid_type == USBHID_TYPE_MOUSE) {
            hid_mouse_close(&devin->mouse);
        } else if (devin->hiddev.hid_type == USBHID_TYPE_KEYBOARD) {
            hid_keyboard_close(&devin->keyboard);
        }
        
        usbhid_close(&devin->hiddev);
        ch375_host_udev_close(&devin->usbdev);
        
        devin->is_connected = 0;
    }
    
    composite_hid_cleanup();
}

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB configured by host");
        composite_hid_set_usb_configured(true);
        composite_hid_send_report_async(0, mouse_report, sizeof(mouse_report));
        break;
    case USB_DC_SUSPEND:
        LOG_INF("USB suspended");
        /* fallthrough */
    case USB_DC_RESET:
    case USB_DC_DISCONNECTED:
        LOG_INF("USB not configured / disconnected");
        composite_hid_set_usb_configured(false);
        break;
    default:
        break;
    }
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
    
    /* Get UART devices using DT macros */
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
    
    /* Initialize AGP context */
    ret = agp_open(&s_agp_ctx);
    if (ret < 0) {
        LOG_ERR("Failed to open AGP context");
        return ret;
    }
    
    /* Main loop - wait for devices and process */
    while (1) {
        LOG_INF("=================================");
        LOG_INF("Waiting for USB devices...");
        LOG_INF("=================================");
        
        /* Wait for both devices to connect */
        wait_all_device_connect();
        
        LOG_INF("=================================");
        LOG_INF("Enumerating devices...");
        LOG_INF("=================================");
        
        /* Enumerate both devices */
        ret = open_all_device_in();
        if (ret < 0) {
            LOG_ERR("Failed to enumerate devices");
            k_msleep(1000);
            continue;
        }
        
        LOG_INF("=================================");
        LOG_INF("Initializing USB device output...");
        LOG_INF("=================================");
        
        /* Register composite HID interfaces */
        open_device_out();
        
        /* Enable USB device */
        ret = usb_enable(usb_status_cb);
        if (ret) {
            LOG_ERR("Failed to enable USB device: %d", ret);
            close_all_devices();
            k_msleep(1000);
            continue;
        }
        
        LOG_INF("=================================");
        LOG_INF("Starting HID processing loop...");
        LOG_INF("=================================");
        
        /* Process HID reports */
        loop_handle_devices();
        
        /* If we get here, a device disconnected */
        LOG_WRN("Device disconnected, restarting...");
        
        /* Disable USB device */
        usb_disable();
        
        /* Close all devices */
        close_all_devices();
        
        k_msleep(1000);
    }
    
    return 0;
}