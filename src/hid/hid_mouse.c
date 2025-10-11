/* HID Mouse Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdlib.h>

#include "ch375/ch375_host.h"
#include "hid/hid_mouse.h"
#include "hid/hid_parser.h"

LOG_MODULE_REGISTER(hid_mouse, LOG_LEVEL_INF);

/* HID Class-Specific Requests */
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_GET_PROTOCOL        0x03
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A
#define HID_SET_PROTOCOL        0x0B
#define HID_REPORT_TYPE_INPUT   0x01
#define HID_REPORT_TYPE_OUTPUT  0x02
#define HID_REPORT_TYPE_FEATURE 0x03

/* Transfer timeout in milliseconds */
#define TRANSFER_TIMEOUT 0

/* USBHID Core Functions Implementation */

void usbhid_free_report_buffer(struct usbhid_device *dev)
{
    if (!dev) {
        return;
    }
    
    if (dev->report_buffer) {
        k_free(dev->report_buffer);
        dev->report_buffer = NULL;
    }
    dev->report_length = 0;
    dev->report_buffer_length = 0;
    dev->report_buffer_last_offset = 0;
}

int usbhid_alloc_report_buffer(struct usbhid_device *dev, uint32_t length)
{
    uint8_t *buf;
    uint32_t buf_len;
    
    if (!dev) {
        LOG_ERR("Invalid device");
        return USBHID_PARAM_INVALID;
    }
    
    if (dev->report_buffer) {
        LOG_ERR("Report buffer already allocated");
        return USBHID_ERROR;
    }
    
    buf_len = length * 2;  /* Double buffer for ping-pong */
    buf = k_malloc(buf_len);
    if (!buf) {
        LOG_ERR("Failed to allocate report buffer (size=%d)", buf_len);
        return USBHID_ALLOC_FAILED;
    }
    
    memset(buf, 0, buf_len);
    
    dev->report_length = length;
    dev->report_buffer = buf;
    dev->report_buffer_length = buf_len;
    dev->report_buffer_last_offset = 0;
    
    return USBHID_SUCCESS;
}

static int usbhid_read(struct usbhid_device *dev, uint8_t *buffer, int length, int *actual_len)
{
    struct usb_device *udev = dev->udev;
    int ret;
    
    ret = ch375_host_interrupt_transfer(udev, dev->ep_in,
                                       buffer, length, actual_len, TRANSFER_TIMEOUT);
    if (ret != CH375_HOST_SUCCESS) {
        if (ret == CH375_HOST_DEV_DISCONNECT) {
            return USBHID_NO_DEV;
        }
        return USBHID_IO_ERROR;
    }
    
    return USBHID_SUCCESS;
}

static inline uint8_t *get_report_buffer(struct usbhid_device *dev, uint8_t is_last)
{
    if (!dev || !dev->report_buffer) {
        return NULL;
    }
    
    if (is_last) {
        return dev->report_buffer + dev->report_buffer_last_offset;
    } else {
        uint32_t offset = dev->report_buffer_last_offset ? 0 : dev->report_length;
        return dev->report_buffer + offset;
    }
}

int usbhid_get_report_buffer(struct usbhid_device *dev, uint8_t **buffer,
                            uint32_t *length, uint8_t is_last)
{
    uint8_t *buf;
    
    if (!dev || !buffer) {
        return USBHID_PARAM_INVALID;
    }
    
    buf = get_report_buffer(dev, is_last);
    if (!buf) {
        LOG_ERR("Report buffer not allocated");
        return USBHID_BUFFER_NOT_ALLOC;
    }
    
    if (buffer) {
        *buffer = buf;
    }
    if (length) {
        *length = dev->report_length;
    }
    
    return USBHID_SUCCESS;
}

int usbhid_fetch_report(struct usbhid_device *dev)
{
    uint8_t *last_report_buffer;
    int actual_len = 0;
    int ret;
    
    if (!dev) {
        return USBHID_PARAM_INVALID;
    }
    
    last_report_buffer = get_report_buffer(dev, 1);
    if (!last_report_buffer) {
        LOG_ERR("Report buffer not allocated");
        return USBHID_BUFFER_NOT_ALLOC;
    }
    
    ret = usbhid_read(dev, last_report_buffer, dev->report_length, &actual_len);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    /* Switch buffer */
    if (dev->report_buffer_last_offset) {
        dev->report_buffer_last_offset = 0;
    } else {
        dev->report_buffer_last_offset = dev->report_length;
    }
    
    return USBHID_SUCCESS;
}

/* Get HID descriptor from device configuration */
static int get_hid_descriptor(struct usb_device *udev, uint8_t interface_num, 
                             struct usb_hid_descriptor **hid_desc)
{
    struct usb_desc_header *desc = (struct usb_desc_header *)udev->raw_conf_desc;
    void *raw_conf_desc_end = (uint8_t *)udev->raw_conf_desc + udev->raw_conf_desc_len;
    uint8_t current_interface_num = 0;
    
    while ((void *)desc < raw_conf_desc_end) {
        desc = (struct usb_desc_header *)((uint8_t *)desc + desc->bLength);
        
        switch (desc->bDescriptorType) {
        case USB_DESC_INTERFACE:
            current_interface_num = ((struct usb_if_descriptor *)desc)->bInterfaceNumber;
            break;
            
        case USB_DESC_HID :
            if (current_interface_num == interface_num) {
                *hid_desc = (struct usb_hid_descriptor *)desc;
                return 0;
            }
            break;
            
        default:
            break;
        }
    }
    
    return -1;
}

/* Get HID class descriptor */
static int hid_get_class_descriptor(struct usb_device *udev, uint8_t interface_num,
                                   uint8_t type, uint8_t *buf, uint16_t len)
{
    uint8_t retries = 4;
    int actual_len = 0;
    int ret;
    
    do {
        ret = ch375_host_control_transfer(udev,
            0x80 | 0x00 | 0x01,
            USB_SREQ_GET_DESCRIPTOR,
            type << 8,
            interface_num,
            buf, len, &actual_len, TRANSFER_TIMEOUT);
            
        if (ret != CH375_HOST_SUCCESS) {
            LOG_ERR("Get class descriptor failed: %d", ret);
        }
        retries--;
    } while (actual_len < len && retries);
    
    if (actual_len < len) {
        LOG_ERR("Insufficient data: actual=%d, expected=%d", actual_len, len);
        return USBHID_ERROR;
    }
    
    return USBHID_SUCCESS;
}

/* Get endpoint from interface */
static int get_ep_in(struct usb_device *udev, uint8_t interface_num, uint8_t *ep)
{
    if (interface_num >= udev->interface_cnt) {
        return -1;
    }
    
    struct usb_interface *interface = &udev->interface[interface_num];
    
    if (interface->endpoint_cnt < 1) {
        LOG_ERR("Interface %d has no endpoints", interface_num);
        return -1;
    }
    
    *ep = interface->endpoint[0].ep_num;
    return 0;
}

/* Set idle request */
static void set_idle(struct usb_device *udev, uint8_t interface_num,
                    uint8_t duration, uint8_t report_id)
{
    int ret;
    
    ret = ch375_host_control_transfer(udev,
        0x00 | 0x20 | 0x01,
        HID_SET_IDLE,
        (duration << 8) | report_id,
        interface_num,
        NULL, 0, NULL, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Set idle failed: %d", ret);
    }
}

/* Set report request */
static int set_report(struct usb_device *udev, uint8_t interface_num,
                      uint8_t report_type, uint8_t report_id)
{
    uint8_t retries = 4;
    int ret;
    int actual_len = 0;
    uint8_t data_fragment = 0x01;
    
    do {
        ret = ch375_host_control_transfer(udev,
            0x00 | 0x20 | 0x01,
            HID_SET_REPORT,
            (report_type << 8) | report_id,
            interface_num,
            &data_fragment, sizeof(data_fragment), &actual_len, TRANSFER_TIMEOUT);
            
        if (ret != CH375_HOST_SUCCESS) {
            LOG_ERR("Set report failed: %d", ret);
        }
        retries--;
    } while (actual_len != sizeof(data_fragment) && retries);
    
    return ret;
}

/* Open USBHID device */
int usbhid_open(struct usb_device *udev, uint8_t interface_num,
               struct usbhid_device *dev)
{
    struct usb_hid_descriptor *desc = NULL;
    uint8_t *raw_hid_report_desc = NULL;
    uint16_t raw_hid_report_desc_len = 0;
    uint8_t hid_type;
    uint8_t ep_in;
    int ret;
    
    ret = get_hid_descriptor(udev, interface_num, &desc);
    if (ret < 0) {
        LOG_ERR("Cannot find HID descriptor for interface %d", interface_num);
        return USBHID_NOT_HID_DEV;
    }
    
    LOG_INF("HID descriptor found: version=0x%04X, country=0x%02X",
           sys_le16_to_cpu(desc->bcdHID), desc->bCountryCode);
    
    if (desc->bNumDescriptors > 1) {
        LOG_ERR("Multiple descriptors not supported: %d", desc->bNumDescriptors);
        return USBHID_NOT_SUPPORT;
    }
    
    memset(dev, 0, sizeof(struct usbhid_device));
    
    set_idle(udev, interface_num, 0, 0);
    LOG_INF("Set idle done");
    
    raw_hid_report_desc_len = sys_le16_to_cpu(desc->wClassDescriptorLength);
    raw_hid_report_desc = k_malloc(raw_hid_report_desc_len);
    if (!raw_hid_report_desc) {
        LOG_ERR("Failed to allocate HID report buffer (len=%d)", raw_hid_report_desc_len);
        return USBHID_ALLOC_FAILED;
    }
    
    memset(raw_hid_report_desc, 0, raw_hid_report_desc_len);
    
    /* Get data endpoint */
    ret = get_ep_in(udev, interface_num, &ep_in);
    if (ret < 0) {
        LOG_ERR("Get endpoint failed for interface %d", interface_num);
        k_free(raw_hid_report_desc);
        return USBHID_NOT_SUPPORT;
    }
    
    /* Get HID report descriptor */
    ret = hid_get_class_descriptor(udev, interface_num, 0x22,
                                 raw_hid_report_desc, raw_hid_report_desc_len);
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("Get HID report descriptor failed: %d", ret);
        k_free(raw_hid_report_desc);
        return USBHID_IO_ERROR;
    }
    
    LOG_INF("Got HID report descriptor (interface=%d, len=%d)", 
           interface_num, raw_hid_report_desc_len);
    
    /* Parse HID report to determine type */
    ret = hid_parse_report_descriptor(raw_hid_report_desc, raw_hid_report_desc_len, &hid_type);
    if (ret < 0) {
        LOG_ERR("Parse HID report failed");
        k_free(raw_hid_report_desc);
        return USBHID_NOT_SUPPORT;
    }
    
    if (hid_type == USBHID_TYPE_KEYBOARD) {
        ret = set_report(udev, interface_num, HID_REPORT_TYPE_OUTPUT, 0);
        if (ret != USBHID_SUCCESS) {
            LOG_ERR("Set report failed");
            k_free(raw_hid_report_desc);
            return USBHID_IO_ERROR;
        }
        LOG_INF("Set report success for keyboard");
    }
    
    dev->udev = udev;
    dev->interface_num = interface_num;
    dev->ep_in = ep_in;
    dev->raw_hid_report_desc = raw_hid_report_desc;
    dev->raw_hid_report_desc_len = raw_hid_report_desc_len;
    dev->hid_desc = desc;
    dev->hid_type = hid_type;
    
    return USBHID_SUCCESS;
}

void usbhid_close(struct usbhid_device *dev)
{
    if (!dev) {
        return;
    }
    
    if (dev->raw_hid_report_desc) {
        k_free(dev->raw_hid_report_desc);
        dev->raw_hid_report_desc = NULL;
    }
    
    usbhid_free_report_buffer(dev);
    memset(dev, 0, sizeof(struct usbhid_device));
}

/* HID Mouse Implementation */

int hid_mouse_get_button(struct hid_mouse *mouse, uint32_t button_num,
                        uint32_t *value, uint8_t is_last)
{
    struct hid_data_descriptor *desc;
    uint8_t *report_buf;
    uint8_t *field_buf;
    uint8_t byte_off = button_num / 8;
    uint8_t bit_off = button_num % 8;
    int ret;
    
    if (!mouse || !value) {
        return USBHID_PARAM_INVALID;
    }
    
    if (button_num >= mouse->button.count) {
        LOG_ERR("Invalid button number: %d", button_num);
        return USBHID_PARAM_INVALID;
    }
    
    ret = usbhid_get_report_buffer(mouse->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    desc = &mouse->button;
    field_buf = report_buf + desc->report_buf_off;
    *value = (field_buf[byte_off] & (0x01 << bit_off)) ? 1 : 0;
    
    return USBHID_SUCCESS;
}

int hid_mouse_set_button(struct hid_mouse *mouse, uint32_t button_num,
                        uint32_t value, uint8_t is_last)
{
    struct hid_data_descriptor *desc;
    uint8_t *report_buf;
    uint8_t *field_buf;
    uint8_t byte_off = button_num / 8;
    uint8_t bit_off = button_num % 8;
    int ret;
    
    if (!mouse) {
        return USBHID_PARAM_INVALID;
    }
    
    if (button_num >= mouse->button.count) {
        LOG_ERR("Invalid button number: %d", button_num);
        return USBHID_PARAM_INVALID;
    }
    
    ret = usbhid_get_report_buffer(mouse->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    desc = &mouse->button;
    field_buf = report_buf + desc->report_buf_off;
    
    if (value) {
        field_buf[byte_off] |= (0x01 << bit_off);
    } else {
        field_buf[byte_off] &= ~(0x01 << bit_off);
    }
    
    return USBHID_SUCCESS;
}

int hid_mouse_get_orientation(struct hid_mouse *mouse, uint32_t axis_num,
                             int32_t *value, uint8_t is_last)
{
    struct hid_data_descriptor *desc;
    uint8_t *report_buf;
    uint8_t *field_buf;
    uint8_t value_byte_size;
    int ret;
    
    if (!mouse || !value) {
        return USBHID_PARAM_INVALID;
    }
    
    if (axis_num >= mouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d", axis_num);
        return USBHID_PARAM_INVALID;
    }
    
    ret = usbhid_get_report_buffer(mouse->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    desc = &mouse->orientation;
    field_buf = report_buf + desc->report_buf_off;
    value_byte_size = desc->size / 8;
    
    switch (value_byte_size) {
    case 1:
        *value = ((int8_t *)field_buf)[axis_num];
        break;
    case 2:
        *value = sys_le16_to_cpu(((int16_t *)field_buf)[axis_num]);
        break;
    case 4:
        *value = sys_le32_to_cpu(((int32_t *)field_buf)[axis_num]);
        break;
    default:
        LOG_ERR("Unexpected value size: %d", value_byte_size);
        return USBHID_ERROR;
    }
    
    return USBHID_SUCCESS;
}

int hid_mouse_set_orientation(struct hid_mouse *mouse, uint32_t axis_num,
                             int32_t value, uint8_t is_last)
{
    struct hid_data_descriptor *desc;
    uint8_t *report_buf;
    uint8_t *field_buf;
    uint8_t value_byte_size;
    int ret;
    
    if (!mouse) {
        return USBHID_PARAM_INVALID;
    }
    
    if (axis_num >= mouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d", axis_num);
        return USBHID_PARAM_INVALID;
    }
    
    ret = usbhid_get_report_buffer(mouse->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        return ret;
    }
    
    desc = &mouse->orientation;
    field_buf = report_buf + desc->report_buf_off;
    value_byte_size = desc->size / 8;
    
    switch (value_byte_size) {
    case 1:
        ((int8_t *)field_buf)[axis_num] = (int8_t)value;
        break;
    case 2:
        ((int16_t *)field_buf)[axis_num] = sys_cpu_to_le16((int16_t)value);
        break;
    case 4:
        ((int32_t *)field_buf)[axis_num] = sys_cpu_to_le32((int32_t)value);
        break;
    default:
        LOG_ERR("Unexpected value size: %d", value_byte_size);
        return USBHID_ERROR;
    }
    
    return USBHID_SUCCESS;
}

int hid_mouse_fetch_report(struct hid_mouse *mouse)
{
    if (!mouse) {
        return USBHID_PARAM_INVALID;
    }
    
    return usbhid_fetch_report(mouse->hid_dev);
}

void hid_mouse_close(struct hid_mouse *mouse)
{
    if (!mouse) {
        return;
    }
    
    usbhid_free_report_buffer(mouse->hid_dev);
    memset(mouse, 0, sizeof(struct hid_mouse));
}

/* Parse HID report for mouse (hardcoded for Logitech G102) */
static int parse_hid_report(struct hid_mouse *mouse, uint8_t *report, uint16_t len)
{
    struct hid_data_descriptor *btn = &mouse->button;
    struct hid_data_descriptor *orien = &mouse->orientation;
    
    /* Hardcoded for Logitech G102 LIGHTSYNC Gaming Mouse */
    /* This should ideally parse the HID report descriptor properly */
    
    mouse->report_length = 8;
    
    /* Button descriptor */
    btn->physical_minimum = 1;
    btn->physical_maximum = 16;
    btn->logical_minimum = 0;
    btn->logical_maximum = 1;
    btn->size = 1;
    btn->count = 16;
    btn->report_buf_off = 0;
    
    /* Orientation descriptor (X/Y axes) */
    orien->physical_minimum = 1;
    orien->physical_maximum = 2;
    orien->logical_minimum = -32767;
    orien->logical_maximum = 32767;
    orien->size = 16;
    orien->count = 2;
    orien->report_buf_off = 2;
    
    return 0;
}

int hid_mouse_open(struct usbhid_device *usbhid_dev, struct hid_mouse *mouse)
{
    int ret;
    
    if (!mouse || !usbhid_dev) {
        LOG_ERR("Invalid parameters");
        return USBHID_PARAM_INVALID;
    }
    
    if (usbhid_dev->hid_type != USBHID_TYPE_MOUSE) {
        LOG_ERR("Not a mouse device");
        return USBHID_NOT_SUPPORT;
    }
    
    memset(mouse, 0, sizeof(struct hid_mouse));
    mouse->hid_dev = usbhid_dev;
    
    ret = parse_hid_report(mouse,
                          usbhid_dev->raw_hid_report_desc,
                          usbhid_dev->raw_hid_report_desc_len);
    if (ret < 0) {
        LOG_ERR("Failed to parse HID report");
        return USBHID_NOT_SUPPORT;
    }
    
    if (mouse->report_length == 0) {
        LOG_ERR("Invalid report length");
        return USBHID_ERROR;
    }
    
    ret = usbhid_alloc_report_buffer(usbhid_dev, mouse->report_length);
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("Failed to allocate report buffer");
        return USBHID_ALLOC_FAILED;
    }
    
    LOG_INF("HID mouse opened (report_len=%d)", mouse->report_length);
    return USBHID_SUCCESS;
}