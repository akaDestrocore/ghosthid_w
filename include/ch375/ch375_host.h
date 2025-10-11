/* CH375 USB Host Mode Header for Zephyr */

#ifndef CH375_HOST_H
#define CH375_HOST_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_ch9.h>
#include "ch375.h"

/* CH375 Host Error Codes */
enum ch375_host_errno {
    CH375_HOST_SUCCESS = 0,
    CH375_HOST_ERROR = -1,
    CH375_HOST_PARAM_INVALID = -2,
    CH375_HOST_TIMEOUT = -3,
    CH375_HOST_DEV_DISCONNECT = -4,
    CH375_HOST_STALL = -5,
    CH375_HOST_IO_ERROR = -6,
    CH375_HOST_NOT_SUPPORT = -7,
    CH375_HOST_ALLOC_FAILED = -8,
};

/* USB Speed */
#define USB_SPEED_LOW           0
#define USB_SPEED_FULL          1
#define USB_SPEED_HIGH          2

/* USB Control Setup Size */
#define CONTROL_SETUP_SIZE      8

/* CH375 Retry Times */
#define CH375_RETRY_TIMES_ZERO      0
#define CH375_RETRY_TIMES_2MS       1
#define CH375_RETRY_TIMES_INFINITY  2

/* USB Endpoint Structure */
struct usb_endpoint {
    uint8_t  ep_num;
    uint8_t  tog;
    uint8_t  attr;
    uint16_t maxpack;
    uint8_t  interval;
};

/* USB Interface Structure */
struct usb_interface {
    uint8_t interface_num;
    uint8_t interface_class;
    uint8_t subclass;
    uint8_t protocol;
    uint8_t endpoint_cnt;
    struct usb_endpoint endpoint[4];
};

/* USB Device Structure - uses Zephyr's descriptors */
struct usb_device {
    struct ch375_context *context;
    
    /* Device info */
    uint16_t vid;
    uint16_t pid;
    uint8_t speed;
    uint8_t ep0_maxpack;
    uint8_t configuration_value;
    
    /* Descriptors - use Zephyr types */
    struct usb_device_descriptor raw_dev_desc;
    uint8_t *raw_conf_desc;
    uint16_t raw_conf_desc_len;
    
    /* Interfaces and endpoints */
    uint8_t interface_cnt;
    struct usb_interface interface[4];
    
    /* Status */
    uint8_t connected;
    uint8_t ready;
};

/* USB HID Descriptor */
struct usb_hid_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bClassDescriptorType;
    uint16_t wClassDescriptorLength;
} __packed;

/* Function prototypes */
int ch375_host_init(struct ch375_context *ctx, uint32_t work_baudrate);
int ch375_host_wait_device_connect(struct ch375_context *ctx, uint32_t timeout_ms);
int ch375_host_udev_open(struct ch375_context *ctx, struct usb_device *udev);
void ch375_host_udev_close(struct usb_device *udev);
int ch375_host_reset_dev(struct usb_device *udev);

/* Transfer functions */
int ch375_host_control_transfer(struct usb_device *udev,
    uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
    uint8_t *data, uint16_t wLength, int *actual_length, uint32_t timeout);

int ch375_host_bulk_transfer(struct usb_device *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout);

int ch375_host_interrupt_transfer(struct usb_device *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout);

int ch375_host_clear_stall(struct usb_device *udev, uint8_t ep);
int ch375_host_set_configuration(struct usb_device *udev, uint8_t configuration);

#endif /* CH375_HOST_H */