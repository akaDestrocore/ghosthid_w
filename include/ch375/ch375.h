/* CH375 Core Driver Header for Zephyr */

#ifndef CH375_H
#define CH375_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* CH375 Commands */
#define CH375_CMD_GET_IC_VER    0x01
#define CH375_CMD_SET_BAUDRATE  0x02
#define CH375_CMD_SET_USB_SPEED 0x04
#define CH375_CMD_CHECK_EXIST   0x06
#define CH375_CMD_GET_DEV_RATE  0x0A
#define CH375_CMD_SET_RETRY     0x0B
#define CH375_CMD_SET_USB_ADDR  0x13
#define CH375_CMD_SET_USB_MODE  0x15
#define CH375_CMD_TEST_CONNECT  0x16
#define CH375_CMD_ABORT_NAK     0x17
#define CH375_CMD_SET_ENDP6     0x1C
#define CH375_CMD_SET_ENDP7     0x1D
#define CH375_CMD_GET_STATUS    0x22
#define CH375_CMD_UNLOCK_USB    0x23
#define CH375_CMD_RD_USB_DATA0  0x27
#define CH375_CMD_RD_USB_DATA   0x28
#define CH375_CMD_WR_USB_DATA7  0x2B
#define CH375_CMD_GET_DESC      0x46
#define CH375_CMD_ISSUE_TKN_X   0x4E
#define CH375_CMD_ISSUE_TOKEN   0x4F

/* CH375 Command Results */
#define CH375_CMD_RET_SUCCESS   0x51
#define CH375_CMD_RET_FAILED    0x5F

/* CH375 USB Modes */
#define CH375_USB_MODE_NO_DETECT    0x04
#define CH375_USB_MODE_NO_SOF       0x05
#define CH375_USB_MODE_SOF          0x06
#define CH375_USB_MODE_RESET        0x07

/* CH375 Interrupt Status */
#define CH375_USB_INT_SUCCESS       0x14
#define CH375_USB_INT_CONNECT       0x15
#define CH375_USB_INT_DISCONNECT    0x16
#define CH375_USB_INT_BUF_OVER      0x17
#define CH375_USB_INT_USB_READY     0x18

/* CH375 PID Codes */
#define CH375_USB_PID_NULL          0x00
#define CH375_USB_PID_SOF           0x05
#define CH375_USB_PID_SETUP         0x0D
#define CH375_USB_PID_IN            0x09
#define CH375_USB_PID_OUT           0x01
#define CH375_USB_PID_ACK           0x02
#define CH375_USB_PID_NAK           0x0A
#define CH375_USB_PID_STALL         0x0E
#define CH375_USB_PID_DATA0         0x03
#define CH375_USB_PID_DATA1         0x0B

/* Macros for command/data differentiation in 9-bit mode */
#define CH375_CMD(x)  ((uint16_t)((x) | 0x0100))
#define CH375_DATA(x) ((uint16_t)((x) | 0x0000))

#define CH375_PID2STATUS(x) ((x) | 0x20)

/* CH375 Error Codes */
enum ch375_errno {
    CH375_SUCCESS = 0,
    CH375_ERROR = -1,
    CH375_PARAM_INVALID = -2,
    CH375_WRITE_CMD_FAILED = -3,
    CH375_READ_DATA_FAILED = -4,
    CH375_NO_EXIST = -5,
    CH375_TIMEOUT = -6,
    CH375_NOT_FOUND = -7,
};

/* CH375 USB Speeds */
#define CH375_USB_SPEED_LOW     1
#define CH375_USB_SPEED_FULL    0

/* CH375 Retry Times */
#define CH375_RETRY_TIMES_ZERO      0x00
#define CH375_RETRY_TIMES_2MS       0x01
#define CH375_RETRY_TIMES_INFINITY  0x02

/* Default Baudrates */
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_WORK_BAUDRATE     115200

/* Forward declaration */
struct ch375_context;

/* Function pointer types for hardware abstraction */
typedef int (*ch375_write_cmd_fn)(struct ch375_context *ctx, uint8_t cmd);
typedef int (*ch375_write_data_fn)(struct ch375_context *ctx, uint8_t data);
typedef int (*ch375_read_data_fn)(struct ch375_context *ctx, uint8_t *data);
typedef int (*ch375_query_int_fn)(struct ch375_context *ctx);

/* CH375 Context Structure */
struct ch375_context {
    void *priv;
    ch375_write_cmd_fn write_cmd;
    ch375_write_data_fn write_data;
    ch375_read_data_fn read_data;
    ch375_query_int_fn query_int;
    struct k_mutex lock;
};

/* CH375 Core Functions */
int ch375_open_context(struct ch375_context **ctx,
                       ch375_write_cmd_fn write_cmd,
                       ch375_write_data_fn write_data,
                       ch375_read_data_fn read_data,
                       ch375_query_int_fn query_int,
                       void *priv);
int ch375_close_context(struct ch375_context *ctx);
void *ch375_get_priv(struct ch375_context *ctx);

/* CH375 Commands */
int ch375_check_exist(struct ch375_context *ctx);
int ch375_get_version(struct ch375_context *ctx, uint8_t *version);
int ch375_set_baudrate(struct ch375_context *ctx, uint32_t baudrate);
int ch375_set_usb_mode(struct ch375_context *ctx, uint8_t mode);
int ch375_get_status(struct ch375_context *ctx, uint8_t *status);
int ch375_abort_nak(struct ch375_context *ctx);
int ch375_query_int(struct ch375_context *ctx);
int ch375_wait_int(struct ch375_context *ctx, uint32_t timeout_ms);

/* Host Mode Commands */
int ch375_test_connect(struct ch375_context *ctx, uint8_t *connect_status);
int ch375_get_dev_speed(struct ch375_context *ctx, uint8_t *speed);
int ch375_set_dev_speed(struct ch375_context *ctx, uint8_t speed);
int ch375_set_usb_addr(struct ch375_context *ctx, uint8_t addr);
int ch375_set_retry(struct ch375_context *ctx, uint8_t times);
int ch375_send_token(struct ch375_context *ctx, uint8_t ep, uint8_t tog,
                    uint8_t pid, uint8_t *status);

/* Data Transfer Commands */
int ch375_write_cmd(struct ch375_context *ctx, uint8_t cmd);
int ch375_write_data(struct ch375_context *ctx, uint8_t data);
int ch375_read_data(struct ch375_context *ctx, uint8_t *data);
int ch375_write_block_data(struct ch375_context *ctx, uint8_t *buf, uint8_t len);
int ch375_read_block_data(struct ch375_context *ctx, uint8_t *buf, uint8_t len,
                         uint8_t *actual_len);

#endif /* CH375_H */