/* include/usb.h -- USB descriptor & constants used by ch375 host code */

#ifndef USB_H
#define USB_H

#include <stdint.h>

#define	USB_PID_NULL	0x00
#define	USB_PID_SOF		0x05
#define	USB_PID_SETUP	0x0D
#define	USB_PID_IN		0x09
#define	USB_PID_OUT		0x01
#define	USB_PID_ACK		0x02
#define	USB_PID_NAK		0x0A
#define	USB_PID_STALL	0x0E
#define	USB_PID_DATA0	0x03
#define	USB_PID_DATA1	0x0B
#define	USB_PID_PRE		0x0C

/* Device and/or Interface Class codes */
enum USB_ClassCode_e {
	USB_CLASS_PER_INTERFACE 		= 0x00,
	USB_CLASS_AUDIO 				= 0x01,
	USB_CLASS_COMM 					= 0x02,
	USB_CLASS_HID 					= 0x03,
	USB_CLASS_PHYSICAL 				= 0x05,
	USB_CLASS_IMAGE 				= 0x06,
	USB_CLASS_PTP 					= 0x06, 	//legacy
	USB_CLASS_PRINTER 				= 0x07,
	USB_CLASS_MASS_STORAGE 			= 0x08,
	USB_CLASS_HUB 					= 0x09,
	USB_CLASS_DATA 					= 0x0a,
	USB_CLASS_SMART_CARD 			= 0x0b,
	USB_CLASS_CONTENT_SECURITY 		= 0x0d,
	USB_CLASS_VIDEO 				= 0x0e,
	USB_CLASS_PERSONAL_HEALTHCARE 	= 0x0f,
	USB_CLASS_DIAGNOSTIC_DEVICE 	= 0xdc,
	USB_CLASS_WIRELESS 				= 0xe0,
	USB_CLASS_MISCELLANEOUS 		= 0xef,
	USB_CLASS_APPLICATION 			= 0xfe,
	USB_CLASS_VENDOR_SPEC 			= 0xff
};

/* Descriptor types as defined by the USB specification. */
enum DescriptorType_e {
	USB_DT_DEVICE 					= 0x01,
	USB_DT_CONFIG 					= 0x02,
	USB_DT_STRING 					= 0x03,
	USB_DT_INTERFACE 				= 0x04,
	USB_DT_ENDPOINT 				= 0x05,
	USB_DT_BOS 						= 0x0f,
	USB_DT_DEVICE_CAPABILITY 		= 0x10,
	USB_DT_HID 						= 0x21,
	USB_DT_REPORT 					= 0x22,
	USB_DT_PHYSICAL 				= 0x23,
	USB_DT_HUB 						= 0x29,
	USB_DT_SUPERSPEED_HUB 			= 0x2a,
	USB_DT_SS_ENDPOINT_COMPANION 	= 0x30
};

enum USB_EndpointTransferType_e {
	USB_ENDPOINT_TRANSFER_TYPE_CONTROL 		= 0x0,
	USB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS 	= 0x1,
	USB_ENDPOINT_TRANSFER_TYPE_BULK 		= 0x2,
	USB_ENDPOINT_TRANSFER_TYPE_INTERRUPT 	= 0x3
};

/* Standard requests (table 9-5 of USB 3.0 specs) */
enum USB_StandardRequest_e {
	USB_REQUEST_GET_STATUS 			= 0x00,
	USB_REQUEST_CLEAR_FEATURE 		= 0x01,
	USB_REQUEST_SET_FEATURE 		= 0x03,
	USB_REQUEST_SET_ADDRESS 		= 0x05,
	USB_REQUEST_GET_DESCRIPTOR 		= 0x06,
	USB_REQUEST_SET_DESCRIPTOR 		= 0x07,
	USB_REQUEST_GET_CONFIGURATION 	= 0x08,
	USB_REQUEST_SET_CONFIGURATION 	= 0x09,
	USB_REQUEST_GET_INTERFACE 		= 0x0a,
	USB_REQUEST_SET_INTERFACE 		= 0x0b,
	USB_REQUEST_SYNCH_FRAME 		= 0x0c,
	USB_REQUEST_SET_SEL 			= 0x30,
	USB_SET_ISOCH_DELAY 			= 0x31
};

/* Endpoint direction (bit 7 of endpoint address) */
enum USB_EndpointDirection_e {
	USB_ENDPOINT_OUT 	= 0x00,
	USB_ENDPOINT_IN 	= 0x80
};

/* bmRequestType: D6~5 */
enum USB_RequestType_e {
	USB_REQUEST_TYPE_STANDARD 	= (0x00 << 5),
	USB_REQUEST_TYPE_CLASS 		= (0x01 << 5),
	USB_REQUEST_TYPE_VENDOR 	= (0x02 << 5),
	USB_REQUEST_TYPE_RESERVED 	= (0x03 << 5)
};

/* bmRequestType: D4~0 */
enum USB_RequestRecipient_e {
	USB_RECIPIENT_DEVICE 		= 0x00,
	USB_RECIPIENT_INTERFACE 	= 0x01,
	USB_RECIPIENT_ENDPOINT 		= 0x02,
	USB_RECIPIENT_OTHER 		= 0x03
};

#define DEVICE_DESC_LEN 0x12
#define CONTROL_SETUP_SIZE sizeof(USB_ControlSetup_t)

#pragma pack(push, 1)
typedef struct USB_ControlSetup_t {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_ControlSetup_t;

typedef struct USB_DeviceDescriptor_t {
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
} USB_DeviceDescriptor_t;

typedef struct USB_Descriptor_t {
    uint8_t bLength;
    uint8_t bDesriptorType;
} USB_Descriptor_t;

typedef struct USB_HID_Descriptor_t_t {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	uint8_t bClassDescriptorType_e;
	uint16_t wClassDescriptorLength;
} USB_HID_Descriptor_t_t;

typedef struct USB_EndpointDescriptor_t {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bEndpointAddress;
	uint8_t  bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t  bInterval;
} USB_EndpointDescriptor_t;

typedef struct USB_InterfaceDescriptor_t {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bInterfaceNumber;
	uint8_t  bAlternateSetting;
	uint8_t  bNumEndpoints;
	uint8_t  bInterfaceClass;
	uint8_t  bInterfaceSubClass;
	uint8_t  bInterfaceProtocol;
	uint8_t  iInterface;
} USB_InterfaceDescriptor_t;

typedef struct USB_ConfigDescriptor_t {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint16_t wTotalLength;
	uint8_t  bNumInterfaces;
	uint8_t  bConfigurationValue;
	uint8_t  iConfiguration;
	uint8_t  bmAttributes;
	uint8_t  MaxPower;
} USB_ConfigDescriptor_t;
#pragma pack(pop)

#endif /* USB_H */
