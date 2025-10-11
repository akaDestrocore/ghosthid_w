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

/**
 * @brief Endpoint direction. Values for bit 7 of the
 */
enum USB_EndpointDirection_e {
	USB_ENDPOINT_OUT 	= 0x00,
	USB_ENDPOINT_IN 	= 0x80
};

/**
 * Device and/or Interface Class codes */
enum USB_ClassCode_e {
	USB_CLASS_PER_INTERFACE 		= 0x00,
	USB_CLASS_AUDIO 				= 0x01,
	USB_CLASS_COMM 					= 0x02,
	USB_CLASS_HID 					= 0x03,
	USB_CLASS_PHYSICAL 				= 0x05,
	USB_CLASS_IMAGE 				= 0x06,
	USB_CLASS_PTP 					= 0x06, 	//legacy name from libusb-0.1 usb.h
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

/**
 * @brief Descriptor types as defined by the USB specification.
 */
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

/**
 * @brief Standard requests, as defined in table 9-5 of the USB 3.0 specifications 
 */
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