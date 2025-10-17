#ifndef CH375_H
#define CH375_H

#include <stdint.h>
#include "ch375_interface.h"

/* CH375 status/return values and API prototypes.
 * This header was trimmed of any platform HAL includes so it can live in Zephyr.
 */

#define CH375_USB_MODE_NO_DETECT    0x04
#define CH375_USB_MODE_NO_SOF       0x05
#define CH375_USB_MODE_SOF          0x06
#define CH375_USB_MODE_RESET        0x07

#define CH375_USB_INT_SUCCESS       0x14
#define CH375_USB_INT_CONNECT       0x15
#define CH375_USB_INT_DISCONNECT    0x16
#define CH375_USB_INT_BUF_OVER      0x17
#define CH375_USB_INT_USB_READY     0x18

#define CH375_CMD_RET_SUCCESS 0x51
#define CH375_CMD_RET_FAILED 0x5F

#define CH375_PID2STATUS(x) ((x) | 0x20)

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

/* public prototypes exposed by the CH375 core implementation (ch375.c) */
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

int ch375_testConnect(CH375_Context_t *context, uint8_t *connect_status);
int ch375_getDevSpeed(CH375_Context_t *context, uint8_t *speed);
int ch375_setDevSpeed(CH375_Context_t *context, uint8_t speed);
int ch375_setUSBAddr(CH375_Context_t *context, uint8_t addr);

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
