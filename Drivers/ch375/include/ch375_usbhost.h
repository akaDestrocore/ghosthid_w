#ifndef CH375_USBHOST_H
#define CH375_USBHOST_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "ch375_interface.h"
#include "usb.h"


#define USB_SPEED_LOW 0
#define USB_SPEED_FULL 1

#define MAX_ENDPOINT_NUM 3
#define MAX_INTERFACE_NUM 3

enum CH375_HST_ERRNO {
    CH375_HST_ERRNO_SUCCESS = 0,
    CH375_HST_ERRNO_ERROR = -1,
    CH375_HST_ERRNO_PARAM_INVALID = -2,
    CH375_HST_ERRNO_SEND_CMD_FAILD = -3,
    CH375_HST_ERRNO_RECV_DATA_FAILD = -4,
    CH375_HST_ERRNO_NO_EXIST = -5,
    CH375_HST_ERRNO_TIMEOUT = -6,
	CH375_HST_ERRNO_DEV_DISCONNECT = -7,
	CH375_HST_ERRNO_STALL = -8,
};

typedef struct USB_Endpoint_t
{
	uint8_t ep_num;   // Endpoint number
	uint8_t tog;      // Synch flag
	uint8_t attr;     // Endpoint properties
	uint8_t maxpack;  // Maximum packet size (up to 64 bytes)
	uint8_t interval;
} USB_Endpoint_t;

typedef struct USB_Interface_t
{
	uint8_t interface_num;
	uint8_t interface_class; 
	uint8_t subclass;
	uint8_t protocol;
	uint8_t endpoint_cnt;
	USB_Endpoint_t endpoint[MAX_ENDPOINT_NUM];
} USB_Interface_t;

typedef struct USB_Device_t
{
	CH375_Context_t *context;
	uint8_t connected;
	uint8_t ready;

	// device
	USB_DeviceDescriptor_t raw_dev_desc;
	uint8_t speed;  // USB_SPEED_LOW
	uint8_t addr;
	uint8_t ep0_maxpack;
	uint16_t vid;
	uint16_t pid;

	// config
	uint8_t *raw_conf_desc;
	uint16_t raw_conf_desc_len;
	uint8_t configuration_value;

	uint8_t interface_cnt;
	USB_Interface_t interface[MAX_INTERFACE_NUM];
} USB_Device_t;

// transfer api
int ch375_hostControlTransfer(USB_Device_t *udev,
	uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
	uint8_t *data, uint16_t wLength, int *actual_length, uint32_t timeout);

int ch375_hostBulkTransfer(USB_Device_t *udev,
	uint8_t ep, uint8_t *data, int length,
	int *actual_length, uint32_t timeout);

int ch375_hostInterruptTransfer(USB_Device_t *udev,
	uint8_t ep, uint8_t *data, int length,
	int *actual_length, uint32_t timeout);

// device operation api
int ch375_hostClearStall(USB_Device_t *udev, uint8_t ep);
int ch375_hostSetConfigration(USB_Device_t *udev, uint8_t iconfigration);
int ch375_hostResetDev(USB_Device_t *udev);

void ch375_hostUdevClose(USB_Device_t *udev);
int ch375_hostUdevOpen(CH375_Context_t *context, USB_Device_t *udev);
int ch375_hostWaitDeviceConnect(CH375_Context_t *context, uint32_t timeout);

int ch375_hostInit(CH375_Context_t *context, uint32_t work_baudrate);

#endif /* CH375_USBHOST_H */