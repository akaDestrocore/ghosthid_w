/* HID Mouse Driver Header for Zephyr */

#ifndef HID_MOUSE_H
#define HID_MOUSE_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include "hid_parser.h"

/* HID Mouse Button Definitions */
#define HID_MOUSE_BUTTON_LEFT   0
#define HID_MOUSE_BUTTON_RIGHT  1
#define HID_MOUSE_BUTTON_MIDDLE 2
#define HID_MOUSE_BUTTON_4      3
#define HID_MOUSE_BUTTON_5      4

/* HID Mouse Axis Definitions */
#define HID_MOUSE_AXIS_X    0
#define HID_MOUSE_AXIS_Y    1
#define HID_MOUSE_AXIS_WHEEL 2

/* USBHID Access Mode */
#define USBHID_NOW  1   /* Get current buffer */
#define USBHID_LAST 0   /* Get last buffer */

/* USBHID Error Codes */
enum usbhid_errno {
    USBHID_SUCCESS = 0,
    USBHID_ERROR = -1,
    USBHID_PARAM_INVALID = -2,
    USBHID_NO_DEV = -3,
    USBHID_IO_ERROR = -4,
    USBHID_NOT_SUPPORT = -5,
    USBHID_NOT_HID_DEV = -6,
    USBHID_BUFFER_NOT_ALLOC = -7,
    USBHID_ALLOC_FAILED = -8,
};

/* USBHID Device Types */
enum usbhid_type {
    USBHID_TYPE_NONE = 0,
    USBHID_TYPE_MOUSE,
    USBHID_TYPE_KEYBOARD,
    USBHID_TYPE_JOYSTICK,
};

/* Forward declarations */
struct usb_device;
struct usb_hid_descriptor;

/* USBHID Device Structure */
struct usbhid_device {
    struct usb_device *udev;
    uint8_t interface_num;
    uint8_t ep_in;
    
    /* HID descriptor and report */
    struct usb_hid_descriptor *hid_desc;
    uint8_t *raw_hid_report_desc;
    uint16_t raw_hid_report_desc_len;
    
    /* Device type */
    enum usbhid_type hid_type;
    
    /* Report buffer */
    uint8_t *report_buffer;
    uint32_t report_length;
    uint32_t report_buffer_length;
    uint32_t report_buffer_last_offset;
};

/* HID Mouse Structure */
struct hid_mouse {
    struct usbhid_device *hid_dev;
    uint32_t report_length;
    
    /* Button descriptor */
    struct hid_data_descriptor button;
    
    /* Orientation descriptor */
    struct hid_data_descriptor orientation;
};

/* HID Mouse Functions */
int hid_mouse_open(struct usbhid_device *hid_dev, struct hid_mouse *mouse);
void hid_mouse_close(struct hid_mouse *mouse);
int hid_mouse_fetch_report(struct hid_mouse *mouse);

/* Button functions */
int hid_mouse_get_button(struct hid_mouse *mouse, uint32_t button_num, 
                        uint32_t *value, uint8_t is_last);
int hid_mouse_set_button(struct hid_mouse *mouse, uint32_t button_num,
                        uint32_t value, uint8_t is_last);

/* Orientation functions */
int hid_mouse_get_orientation(struct hid_mouse *mouse, uint32_t axis_num,
                             int32_t *value, uint8_t is_last);
int hid_mouse_set_orientation(struct hid_mouse *mouse, uint32_t axis_num,
                             int32_t value, uint8_t is_last);

/* USBHID Core Functions */
int usbhid_open(struct usb_device *udev, uint8_t interface_num,
               struct usbhid_device *dev);
void usbhid_close(struct usbhid_device *dev);
int usbhid_fetch_report(struct usbhid_device *dev);
int usbhid_get_report_buffer(struct usbhid_device *dev, uint8_t **buffer,
                            uint32_t *length, uint8_t is_last);
int usbhid_alloc_report_buffer(struct usbhid_device *dev, uint32_t length);
void usbhid_free_report_buffer(struct usbhid_device *dev);

#endif /* HID_MOUSE_H */