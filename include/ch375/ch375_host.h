/* CH375 USB Host Mode Header for Zephyr */

#ifndef CH375_HOST_H
#define CH375_HOST_H

#include <stdint.h>
#include <zephyr/kernel.h>
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

/* USB Standard Definitions */
#define USB_ENDPOINT_IN         0x80
#define USB_ENDPOINT_OUT        0x00

/* USB Request Types */
#define USB_REQUEST_TYPE_STANDARD   0x00
#define USB_REQUEST_TYPE_CLASS      0x20
#define USB_REQUEST_TYPE_VENDOR     0x40

/* USB Recipients */
#define USB_RECIPIENT_DEVICE        0x00
#define USB_RECIPIENT_INTERFACE     0x01
#define USB_RECIPIENT_ENDPOINT      0x02

/* USB Standard Requests */
#define USB_REQUEST_GET_STATUS          0x00
#define USB_REQUEST_CLEAR_FEATURE       0x01
#define USB_REQUEST_SET_FEATURE         0x03
#define USB_REQUEST_SET_ADDRESS         0x05
#define USB_REQUEST_GET_DESCRIPTOR      0x06
#define USB_REQUEST_SET_DESCRIPTOR      0x07
#define USB_REQUEST_GET_CONFIGURATION   0x08
#define USB_REQUEST_SET_CONFIGURATION   0x09

/* USB Descriptor Types */
#define USB_DT_DEVICE           0x01
#define USB_DT_CONFIG           0x02
#define USB_DT_STRING           0x03
#define USB_DT_INTERFACE        0x04
#define USB_DT_ENDPOINT         0x05
#define USB_DT_HID              0x21
#define USB_DT_REPORT           0x22
#define USB_DT_PHYSICAL         0x23

/* USB Speed */
#define USB_SPEED_LOW           0
#define USB_SPEED_FULL          1
#define USB_SPEED_HIGH          2

/* USB Transfer Types */
#define USB_ENDPOINT_TRANSFER_TYPE_CONTROL     0x00
#define USB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS 0x01
#define USB_ENDPOINT_TRANSFER_TYPE_BULK        0x02
#define USB_ENDPOINT_TRANSFER_TYPE_INTERRUPT   0x03

/* USB Control Setup Size */
#define CONTROL_SETUP_SIZE      8

/* CH375 Retry Times */
#define CH375_RETRY_TIMES_ZERO      0
#define CH375_RETRY_TIMES_2MS       1
#define CH375_RETRY_TIMES_INFINITY  2

/* USB Descriptors */
struct usb_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
} __packed;

struct usb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __packed;

struct usb_config_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __packed;

struct usb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __packed;

struct usb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __packed;

struct usb_control_setup {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __packed;

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

/* USB Device Structure */
struct usb_device {
    struct ch375_context *context;
    
    /* Device info */
    uint16_t vid;
    uint16_t pid;
    uint8_t speed;
    uint8_t ep0_maxpack;
    uint8_t configuration_value;
    
    /* Descriptors */
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