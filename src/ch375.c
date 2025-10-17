/* src/ch375.c
 *
 * Zephyr-adapted CH375 core. Based on Drivers/ch375/src/ch375.c but
 * replaced HAL calls and stdlib functions with Zephyr APIs.
 *
 * Important: this file expects the low-level I/O to be supplied by the
 * CH375 context write_cmd/write_data/read_data/query_int function pointers.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>

#include "ch375.h"
#include "ch375_interface.h"

LOG_MODULE_REGISTER(ch375, LOG_LEVEL_INF);

/* local convenience: millisecond sleep wrapper */
static inline void _msleep(uint32_t ms)
{
    if (ms == 0) {
        return;
    }
    /* k_msleep is safe even in threads */
    k_msleep(ms);
}

/* timeouts in ms used in original code; WAIT_INT_TIMEOUT_MS defined there */
#define WAIT_INT_TIMEOUT_MS (2 * 1000)

/* commands from original */
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

#define CH375_CMD_RET_SUCCESS 0x51
#define CH375_CMD_RET_FAILED  0x5F

#define ch375_checkExist_DATA1 0x65
#define CH375_CHECK_EIXST_DATA2 ((uint8_t)~ch375_checkExist_DATA1)

#define CH375_GET_DEV_RATE_DATA 0x07

#define CH375_SET_USB_SPEED_LOW 0x02
#define CH375_SET_USB_SPEED_FULL 0x00

#define ch375_setRetry_DATA 0x25

/* The context struct is defined in implementation (CH375 interface typedef exists) */
struct CH375_Context_t {
    void *priv;
    func_write_cmd write_cmd;
    func_write_data write_data;
    func_read_data read_data;
    func_query_int query_int;
};

int ch375_writeCmd(CH375_Context_t *context, uint8_t cmd)
{
    if (context == NULL) {
        LOG_ERR("ch375_writeCmd: null context");
        return CH375_PARAM_INVALID;
    }

    return context->write_cmd(context, cmd);
}

int ch375_write_data(CH375_Context_t *context, uint8_t data)
{
    if (context == NULL) {
        LOG_ERR("ch375_write_data: null context");
        return CH375_PARAM_INVALID;
    }

    return context->write_data(context, data);
}

int ch375_readData(CH375_Context_t *context, uint8_t *data)
{
    if (data == NULL) {
        LOG_ERR("ch375_readData: null data ptr");
        return CH375_PARAM_INVALID;
    }
    return context->read_data(context, data);
}

int ch375_writeBlockData(CH375_Context_t *context, uint8_t *buf, uint8_t len)
{
    int ret;
    uint8_t offset;

    ret = ch375_writeCmd(context, CH375_CMD_WR_USB_DATA7);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, len);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    offset = 0;
    while (len > 0) {
        ret = ch375_write_data(context, *(buf + offset));
        if (ret != CH375_SUCCESS) {
            return ch375_writeCmd_FAILD;
        }
        offset++;
        len--;
    }
    return CH375_SUCCESS;
}

int ch375_readBlockData(CH375_Context_t *context, uint8_t *buf, uint8_t len, uint8_t *actual_len)
{
    int ret;
    uint8_t data_len;
    uint8_t residue_len;
    uint8_t offset;

    if (buf == NULL) {
        LOG_ERR("ch375_readBlockData: buf NULL");
        return CH375_PARAM_INVALID;
    }
    if (actual_len == NULL) {
        LOG_ERR("ch375_readBlockData: actual_len NULL");
        return CH375_PARAM_INVALID;
    }

    ret = ch375_writeCmd(context, CH375_CMD_RD_USB_DATA);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    ret = ch375_readData(context, &data_len);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }
    residue_len = data_len;

    offset = 0;
    while (residue_len > 0) {
        ret = ch375_readData(context, buf + offset);
        if (ret != CH375_SUCCESS) {
            return ch375_readData_FAILD;
        }
        offset++;
        residue_len--;
    }
    LOG_DBG("ch375_readBlockData actual_len=%d", offset);
    *actual_len = offset;
    return CH375_SUCCESS;
}

int ch375_getVersion(CH375_Context_t *context, uint8_t *version)
{
    uint8_t buf = 0;
    int ret = -1;

    if (version == NULL) {
        LOG_ERR("ch375_getVersion: version NULL");
        return CH375_PARAM_INVALID;
    }

    ret = ch375_writeCmd(context, CH375_CMD_GET_IC_VER);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_readData(context, &buf);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }
    /* lower 6 bits is version, ref CH375 datasheet */
    *version = 0x3F & buf;
    return CH375_SUCCESS;
}

int ch375_setBaudrate(CH375_Context_t *context, uint32_t baudrate)
{
    int ret;
    uint8_t data1;
    uint8_t data2;

    if (baudrate == 9600) {
        data1 = 0x02;
        data2 = 0xB2;
    } else if (baudrate == 115200) {
        data1 = 0x03;
        data2 = 0xCC;
    } else {
        LOG_ERR("ch375_setBaudrate: unsupported %lu", (unsigned long)baudrate);
        return CH375_PARAM_INVALID;
    }

    ret = ch375_writeCmd(context, CH375_CMD_SET_BAUDRATE);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, data1);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, data2);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    return CH375_SUCCESS;
}

int ch375_queryInt(CH375_Context_t *context)
{
    return context->query_int(context);
}

int ch375_waitInt(CH375_Context_t *context, uint32_t timeout)
{
    uint32_t cnt;

    for (cnt = 0; cnt < timeout; cnt++) {
        if (ch375_queryInt(context)) {
            return CH375_SUCCESS;
        }
        _msleep(1);
    }
    return CH375_TIMEOUT;
}

int ch375_checkExist(CH375_Context_t *context)
{
    uint8_t recv_buf = 0;
    int ret = ch375_writeCmd(context, CH375_CMD_CHECK_EXIST);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, ch375_checkExist_DATA1);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_readData(context, &recv_buf);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }
    if (recv_buf != CH375_CHECK_EIXST_DATA2) {
        LOG_ERR("ch375_checkExist: receive 0x%02X expected 0x%02X", recv_buf, CH375_CHECK_EIXST_DATA2);
        return CH375_NO_EXIST;
    }
    return CH375_SUCCESS;
}

int ch375_setUSBMode(CH375_Context_t *context, uint8_t mode)
{
    uint8_t buf = 0;
    int ret;

    ret = ch375_writeCmd(context, CH375_CMD_SET_USB_MODE);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, mode);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    /* original comment: 20us to set success, 1ms is enough (we keep 1ms) */
    _msleep(1);

    ret = ch375_readData(context, &buf);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }

    if (buf != CH375_CMD_RET_SUCCESS) {
        LOG_ERR("ch375_setUSBMode failed, ret code=0x%02X", buf);
        return CH375_ERROR;
    }
    return CH375_SUCCESS;
}

int ch375_setUSBAddr(CH375_Context_t *context, uint8_t addr)
{
    int ret;

    ret = ch375_writeCmd(context, CH375_CMD_SET_USB_ADDR);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, addr);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    return CH375_SUCCESS;
}

int ch375_getDevSpeed(CH375_Context_t *context, uint8_t *speed)
{
    uint8_t buf = 0;
    int ret;

    ret = ch375_writeCmd(context, CH375_CMD_GET_DEV_RATE);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, CH375_GET_DEV_RATE_DATA);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    ret = ch375_readData(context, &buf);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }
    if (buf & 0x10) {
        *speed = CH375_USB_SPEED_LOW;
    } else {
        *speed = CH375_USB_SPEED_FULL;
    }
    return CH375_SUCCESS;
}

int ch375_setDevSpeed(CH375_Context_t *context, uint8_t speed)
{
    int ret;
    uint8_t val;

    if (speed != CH375_USB_SPEED_LOW && speed != CH375_USB_SPEED_FULL) {
        LOG_ERR("ch375_setDevSpeed invalid speed 0x%02X", speed);
        return CH375_PARAM_INVALID;
    }

    val = (speed == CH375_USB_SPEED_LOW) ? CH375_SET_USB_SPEED_LOW : CH375_SET_USB_SPEED_FULL;

    ret = ch375_writeCmd(context, CH375_CMD_SET_USB_SPEED);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, val);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    return CH375_SUCCESS;
}

int ch375_testConnect(CH375_Context_t *context, uint8_t *connect_status)
{
    int ret;
    uint8_t buf;
    uint8_t status;

    if (connect_status == NULL) {
        LOG_ERR("ch375_testConnect param null");
        return CH375_PARAM_INVALID;
    }

    ret = ch375_writeCmd(context, CH375_CMD_TEST_CONNECT);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    _msleep(1);

    ret = ch375_readData(context, &buf);
    if (ret != CH375_SUCCESS) {
        return ch375_readData_FAILD;
    }

    LOG_DBG("ch375_testConnect status=0x%02X", buf);
    if (buf != CH375_USB_INT_DISCONNECT &&
        buf != CH375_USB_INT_CONNECT &&
        buf != CH375_USB_INT_USB_READY) {
        buf = CH375_USB_INT_DISCONNECT;
    }

    if (buf == CH375_USB_INT_DISCONNECT) {
        (void)ch375_getStatus(context, &status);
    }

    *connect_status = buf;
    return CH375_SUCCESS;
}

int ch375_getStatus(CH375_Context_t *context, uint8_t *status)
{
    uint8_t buf = 0;
    int ret = -1;

    if (status == NULL) {
        LOG_ERR("ch375_getStatus param null");
        return CH375_PARAM_INVALID;
    }

    ret = ch375_writeCmd(context, CH375_CMD_GET_STATUS);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    ret = ch375_readData(context, &buf);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("ch375_getStatus readData failed");
        return ch375_readData_FAILD;
    }

    *status = buf;
    return CH375_SUCCESS;
}

int ch375_abortNAK(CH375_Context_t *context)
{
    int ret;
    ret = ch375_writeCmd(context, CH375_CMD_ABORT_NAK);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    return CH375_SUCCESS;
}

int ch375_setRetry(CH375_Context_t *context, uint8_t times)
{
    int ret;
    uint8_t param;
    ret = ch375_writeCmd(context, CH375_CMD_SET_RETRY);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    ret = ch375_write_data(context, ch375_setRetry_DATA);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    if (times == 0) {
        param = 0x05;
    } else if (times == 1) {
        param = 0xC0;
    } else {
        param = 0x85;
    }

    ret = ch375_write_data(context, param);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    return CH375_SUCCESS;
}

int ch375_sendToken(CH375_Context_t *context, uint8_t ep, uint8_t tog, uint8_t pid, uint8_t *status)
{
    int ret;
    uint8_t tog_val;
    uint8_t ep_pid;
    uint8_t st;

    tog_val = tog ? 0xC0 : 0x00;
    ep_pid = (ep << 4) | pid;

    LOG_DBG("ch375_sendToken tog=0x%02X ep_pid=0x%02X", tog_val, ep_pid);

    ret = ch375_writeCmd(context, CH375_CMD_ISSUE_TKN_X);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, tog_val);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }
    ret = ch375_write_data(context, ep_pid);
    if (ret != CH375_SUCCESS) {
        return ch375_writeCmd_FAILD;
    }

    /* wait interrupt and read status */
    ret = ch375_waitInt(context, WAIT_INT_TIMEOUT_MS);
    if (ret != CH375_SUCCESS) {
        return CH375_TIMEOUT;
    }
    ret = ch375_getStatus(context, &st);
    if (ret != CH375_SUCCESS) {
        return CH375_ERROR;
    }

    *status = st;
    return CH375_SUCCESS;
}

/* sanity check helper */
static int check_context_invalid(CH375_Context_t *context)
{
    if (!context) {
        LOG_ERR("check_context_invalid: context NULL");
        return CH375_ERROR;
    }
    if (context->query_int == NULL) {
        LOG_ERR("context->query_int is NULL");
        goto failed;
    }
    if (context->write_cmd == NULL) {
        LOG_ERR("context->write_cmd is NULL");
        goto failed;
    }
    if (context->read_data == NULL) {
        LOG_ERR("context->read_data is NULL");
        goto failed;
    }
    if (context->write_data == NULL) {
        LOG_ERR("context->write_data is NULL");
        goto failed;
    }
    return CH375_SUCCESS;
failed:
    return CH375_ERROR;
}

void *ch375_getPriv(CH375_Context_t *context)
{
    if (context == NULL) {
        return NULL;
    }
    return context->priv;
}

int ch375_closeContext(CH375_Context_t *context)
{
    if (context == NULL) {
        LOG_ERR("ch375_closeContext: null context");
        return CH375_PARAM_INVALID;
    }
    memset(context, 0, sizeof(CH375_Context_t));
    k_free(context);
    return CH375_SUCCESS;
}

int ch375_openContext(CH375_Context_t **context,
    func_write_cmd write_cmd,
    func_write_data write_data,
    func_read_data read_data,
    func_query_int query_int,
    void *priv)
{
    CH375_Context_t *ctx;
    if (context == NULL) {
        LOG_ERR("ch375_openContext: null out ptr");
        return CH375_PARAM_INVALID;
    }

    ctx = (CH375_Context_t *)k_malloc(sizeof(CH375_Context_t));
    if (!ctx) {
        LOG_ERR("ch375_openContext: alloc failed");
        return CH375_ERROR;
    }
    memset(ctx, 0, sizeof(CH375_Context_t));

    ctx->priv = priv;
    ctx->write_cmd = write_cmd;
    ctx->write_data = write_data;
    ctx->read_data = read_data;
    ctx->query_int = query_int;

    if (check_context_invalid(ctx) != CH375_SUCCESS) {
        LOG_ERR("ch375_openContext: check_context_invalid");
        k_free(ctx);
        return CH375_PARAM_INVALID;
    }

    *context = ctx;
    return CH375_SUCCESS;
}
