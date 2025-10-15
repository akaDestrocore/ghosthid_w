#ifndef CH375_H
#define CH375_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "ch375_interface.h"

/**
 * @brief CH375 USB mode
 * @note 00H switches to inactive USB device mode (the default mode after power-on or reset).
        01H switches to active USB device mode (external firmware mode).
        02H switches to active USB device mode (internal firmware mode).
        04H switches to inactive USB host mode.
        05H switches to active USB host mode (no SOF packet generation).
        06H switches to active USB host mode (automatically generating a SOF packet).
        07H switches to active USB host mode (resetting the USB bus).

        It is recommended to use mode 5 when no USB device is present. After inserting a USB device, first enter mode 7 and then switch to mode 6.

        Usually, setting the USB operating mode is completed within 20us, and the operating status is output upon completion.
 */
#define CH375_USB_MODE_NO_DETECT    0x04
#define CH375_USB_MODE_NO_SOF       0x05
#define CH375_USB_MODE_SOF          0x06
#define CH375_USB_MODE_RESET        0x07

/**
 * @brief Host mode, int status code
 * @note    * Bits 7-6 are 00 
 *          * Bit 5 is 1
 *          * Bit 4 indicates whether the currently received data packet is synchronized
 *          * Bits 3-0 indicate the USB device's response when communication fails: 0010 = ACK, 
 *                      1010 = NAK, 1110 = STALL, 0011 = DATA0, 1011 = DATA1, XX00 = Timeout
 *          * USB_INT_RET_ACK 0x001X 0010B 
 *              Error: Return ACK for IN transaction
 *          * USB_INT_RET_NAK 0x001X 1010B
 *              Error: Return NAK
 *          * USB_INT_RET_NAK 0x001X 1010B
 *              Error: Return NAK
 *          * USB_INT_RET_STALL 0x001X 1110B
 *              Error: Return STALL
 *          * USB_INT_RET_DATA0 0x001X 0011B 
 *              Error: DATA0 returned for OUT/SETUP transaction
 *          * USB_INT_RET_DATA1 0x001X 1011B
 *              Error: DATA1 returned for OUT/SETUP transaction
 *          * USB_INT_RET_TOUT 0x001X XX00B
 *              Error: Return timeout
 *          * USB_INT_RET_TOGX 0x0010 X011B
 *              Error: Data returned for IN transaction out of sync
 *          * USB_INT_RET_PID 0x001X XXXXB
 *              Error: Undefined
 */
 
#define	CH375_USB_INT_SUCCESS       0x14			// USB transaction successful
#define	CH375_USB_INT_CONNECT		0x15			// USB device connection event detected
#define	CH375_USB_INT_DISCONNECT	0x16			// USB device disconnection event detected
#define	CH375_USB_INT_BUF_OVER	    0x17			// USB buffer overflow
#define	CH375_USB_INT_USB_READY	    0x18			// USB address has been assigned

 /**
  * @brief   USB packet identification PID
  */
#define	CH375_USB_PID_NULL 0x00
#define	CH375_USB_PID_SOF 0x05
#define	CH375_USB_PID_SETUP	0x0D
#define	CH375_USB_PID_IN 0x09
#define	CH375_USB_PID_OUT 0x01
#define	CH375_USB_PID_ACK 0x02
#define	CH375_USB_PID_NAK 0x0A
#define	CH375_USB_PID_STALL	0x0E
#define	CH375_USB_PID_DATA0	0x03
#define	CH375_USB_PID_DATA1	0x0B
#define	CH375_USB_PID_PRE 0x0C

#define CH375_PID2STATUS(x) ((x) | 0x20)

/**
 * @brief 
 */
#define CH375_USB_DESC_TYPE_DEV_DESC    0x01
#define CH375_USB_DESC_TYPE_CONFIG_DESC 0x02

#define CH375_USB_SPEED_LOW  1
#define CH375_USB_SPEED_FULL 0

#define ch375_setRetry_TIMES_ZROE      0x00
#define ch375_setRetry_TIMES_2MS       0x01
#define ch375_setRetry_TIMES_INFINITY  0x02

/**
 * @brief Any Mode
 * @param baudrate, support 9600, 115200
 */
int ch375_setBaudrate(CH375_Context_t *context, uint32_t baudrate);
int ch375_queryInt(CH375_Context_t *context);
int ch375_waitInt(CH375_Context_t *context, uint32_t timeout);

int ch375_writeCmd(CH375_Context_t *context, uint8_t cmd);
int ch375_readData(CH375_Context_t *context, uint8_t *data);
int ch375_writeBlockData(CH375_Context_t *context, uint8_t *buf, uint8_t len);
int ch375_readBlockData(CH375_Context_t *context, uint8_t *buf, uint8_t len, uint8_t *actual_len);


int ch375_checkExist(CH375_Context_t *context);
int ch375_getVersion(CH375_Context_t *context, uint8_t *version);
int ch375_setUSBMode(CH375_Context_t *context, uint8_t mode);
int ch375_getStatus(CH375_Context_t *context, uint8_t *status);
int ch375_abortNAK(CH375_Context_t *context);

/**
 * @brief   Host Mode
 */
int ch375_testConnect(CH375_Context_t *context, uint8_t *connect_status);
int ch375_getDevSpeed(CH375_Context_t *context, uint8_t *speed);
int ch375_setDevSpeed(CH375_Context_t *context, uint8_t speed);
int ch375_setUSBAddr(CH375_Context_t *context, uint8_t addr); // just tell deivce address to ch375

/**
 * @brief 
 * @param context 
 * @param times ch375_setRetry_TIMES_ZROE, ch375_setRetry_TIMES_2MS, ch375_setRetry_TIMES_INFINITY
 */
int ch375_setRetry(CH375_Context_t *context, uint8_t times);
int ch375_sendToken(CH375_Context_t *context, uint8_t ep, uint8_t tog, uint8_t pid, uint8_t *status);


void *ch375_getPriv(CH375_Context_t *context);

int ch375_closeContext(CH375_Context_t *context);
int ch375_openContext(CH375_Context_t **context, 
    func_write_cmd write_cmd,
    func_write_data write_data,
    func_read_data read_data,
    func_query_int query_int,
    void *priv);

#endif /* CH375_H */