/* CH375 Core Driver Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

#include "ch375/ch375.h"

LOG_MODULE_REGISTER(ch375, LOG_LEVEL_DBG);

#define WAIT_INT_TIMEOUT_MS 2000
#define CH375_CHECK_EXIST_DATA1 0x65
#define CH375_CHECK_EXIST_DATA2 ((uint8_t)~CH375_CHECK_EXIST_DATA1)

/* Implementation of core CH375 functions */

int ch375_write_cmd(struct ch375_context *ctx, uint8_t cmd)
{
    if (!ctx) {
        LOG_ERR("Invalid context");
        return CH375_PARAM_INVALID;
    }
    
    return ctx->write_cmd(ctx, cmd);
}

int ch375_write_data(struct ch375_context *ctx, uint8_t data)
{
    if (!ctx) {
        LOG_ERR("Invalid context");
        return CH375_PARAM_INVALID;
    }
    
    return ctx->write_data(ctx, data);
}

int ch375_read_data(struct ch375_context *ctx, uint8_t *data)
{
    if (!ctx || !data) {
        LOG_ERR("Invalid parameters");
        return CH375_PARAM_INVALID;
    }
    
    return ctx->read_data(ctx, data);
}

int ch375_write_block_data(struct ch375_context *ctx, uint8_t *buf, uint8_t len)
{
    int ret;
    uint8_t offset;
    
    if (!ctx || !buf) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_WR_USB_DATA7);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, len);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    offset = 0;
    while (len > 0) {
        ret = ch375_write_data(ctx, buf[offset]);
        if (ret != CH375_SUCCESS) {
            k_mutex_unlock(&ctx->lock);
            return CH375_WRITE_CMD_FAILED;
        }
        offset++;
        len--;
    }
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_read_block_data(struct ch375_context *ctx, uint8_t *buf, uint8_t len,
                         uint8_t *actual_len)
{
    int ret;
    uint8_t data_len;
    uint8_t residue_len;
    uint8_t offset;
    
    if (!ctx || !buf || !actual_len) {
        LOG_ERR("Invalid parameters");
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_RD_USB_DATA);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_read_data(ctx, &data_len);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    residue_len = data_len;
    offset = 0;
    
    while (residue_len > 0 && offset < len) {
        ret = ch375_read_data(ctx, &buf[offset]);
        if (ret != CH375_SUCCESS) {
            k_mutex_unlock(&ctx->lock);
            return CH375_READ_DATA_FAILED;
        }
        offset++;
        residue_len--;
    }
    
    *actual_len = offset;
    
    k_mutex_unlock(&ctx->lock);
    LOG_DBG("Read block data: actual_len=%d", offset);
    
    return CH375_SUCCESS;
}

int ch375_get_version(struct ch375_context *ctx, uint8_t *version)
{
    uint8_t buf = 0;
    int ret;
    
    if (!ctx || !version) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_GET_IC_VER);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_read_data(ctx, &buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    *version = buf & 0x3F;  /* Lower 6 bits is version */
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_set_baudrate(struct ch375_context *ctx, uint32_t baudrate)
{
    int ret;
    uint8_t data1, data2;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    if (baudrate == 9600) {
        data1 = 0x02;
        data2 = 0xB2;
    } else if (baudrate == 115200) {
        data1 = 0x03;
        data2 = 0xCC;
    } else {
        LOG_ERR("Unsupported baudrate: %u", baudrate);
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_SET_BAUDRATE);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, data1);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, data2);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_query_int(struct ch375_context *ctx)
{
    if (!ctx) {
        return 0;
    }
    
    return ctx->query_int(ctx);
}

int ch375_wait_int(struct ch375_context *ctx, uint32_t timeout_ms)
{
    uint32_t start = k_uptime_get_32();
    
    while ((k_uptime_get_32() - start) < timeout_ms) {
        if (ch375_query_int(ctx)) {
            return CH375_SUCCESS;
        }
        k_msleep(1);
    }
    
    return CH375_TIMEOUT;
}

int ch375_check_exist(struct ch375_context *ctx)
{
    uint8_t recv_buf = 0;
    int ret;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_CHECK_EXIST);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, CH375_CHECK_EXIST_DATA1);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_read_data(ctx, &recv_buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    
    if (recv_buf != CH375_CHECK_EXIST_DATA2) {
        LOG_ERR("Check exist failed: expected 0x%02X, got 0x%02X",
               CH375_CHECK_EXIST_DATA2, recv_buf);
        return CH375_NO_EXIST;
    }
    
    return CH375_SUCCESS;
}

int ch375_set_usb_mode(struct ch375_context *ctx, uint8_t mode)
{
    uint8_t buf = 0;
    int ret;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_SET_USB_MODE);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, mode);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_msleep(1);  /* Wait for mode change */
    
    ret = ch375_read_data(ctx, &buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    
    if (buf != CH375_CMD_RET_SUCCESS) {
        LOG_ERR("Set USB mode failed: ret=0x%02X", buf);
        return CH375_ERROR;
    }
    
    return CH375_SUCCESS;
}

int ch375_get_status(struct ch375_context *ctx, uint8_t *status)
{
    uint8_t buf = 0;
    int ret;
    
    if (!ctx || !status) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_GET_STATUS);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_read_data(ctx, &buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    *status = buf;
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_abort_nak(struct ch375_context *ctx)
{
    int ret;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_ABORT_NAK);
    
    k_mutex_unlock(&ctx->lock);
    
    return ret == CH375_SUCCESS ? CH375_SUCCESS : CH375_WRITE_CMD_FAILED;
}

/* Host mode specific functions */
int ch375_test_connect(struct ch375_context *ctx, uint8_t *connect_status)
{
    int ret;
    uint8_t buf;
    uint8_t status;
    
    if (!ctx || !connect_status) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_TEST_CONNECT);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_msleep(1);
    
    ret = ch375_read_data(ctx, &buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    
    LOG_DBG("Test connect status=0x%02X", buf);
    
    if (buf != CH375_USB_INT_DISCONNECT &&
        buf != CH375_USB_INT_CONNECT &&
        buf != CH375_USB_INT_USB_READY) {
        buf = CH375_USB_INT_DISCONNECT;
    }
    
    if (buf == CH375_USB_INT_DISCONNECT) {
        ch375_get_status(ctx, &status);
    }
    
    *connect_status = buf;
    return CH375_SUCCESS;
}

int ch375_set_usb_addr(struct ch375_context *ctx, uint8_t addr)
{
    int ret;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_SET_USB_ADDR);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, addr);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_get_dev_speed(struct ch375_context *ctx, uint8_t *speed)
{
    uint8_t buf = 0;
    int ret;
    
    if (!ctx || !speed) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_GET_DEV_RATE);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, 0x07);  /* CH375_GET_DEV_RATE_DATA */
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_read_data(ctx, &buf);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_READ_DATA_FAILED;
    }
    
    *speed = (buf & 0x10) ? CH375_USB_SPEED_LOW : CH375_USB_SPEED_FULL;
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_set_dev_speed(struct ch375_context *ctx, uint8_t speed)
{
    int ret;
    uint8_t val;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    if (speed != CH375_USB_SPEED_LOW && speed != CH375_USB_SPEED_FULL) {
        LOG_ERR("Invalid speed: 0x%02X", speed);
        return CH375_PARAM_INVALID;
    }
    
    val = (speed == CH375_USB_SPEED_LOW) ? 0x02 : 0x00;
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_SET_USB_SPEED);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, val);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_set_retry(struct ch375_context *ctx, uint8_t times)
{
    int ret;
    uint8_t param;
    
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_SET_RETRY);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, 0x25);  /* CH375_SET_RETRY_DATA */
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    if (times == 0) {
        param = 0x05;  /* NAK no retry */
    } else if (times == 1) {
        param = 0xC0;  /* Retry 200ms-2s */
    } else {
        param = 0x85;  /* Infinite retry */
    }
    
    ret = ch375_write_data(ctx, param);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    return CH375_SUCCESS;
}

int ch375_send_token(struct ch375_context *ctx, uint8_t ep, uint8_t tog,
                    uint8_t pid, uint8_t *status)
{
    int ret;
    uint8_t tog_val;
    uint8_t ep_pid;
    uint8_t st;
    
    if (!ctx || !status) {
        return CH375_PARAM_INVALID;
    }
    
    tog_val = tog ? 0xC0 : 0x00;
    ep_pid = (ep << 4) | pid;
    
    LOG_DBG("Send token: tog=0x%02X, ep_pid=0x%02X", tog_val, ep_pid);
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ret = ch375_write_cmd(ctx, CH375_CMD_ISSUE_TKN_X);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, tog_val);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    ret = ch375_write_data(ctx, ep_pid);
    if (ret != CH375_SUCCESS) {
        k_mutex_unlock(&ctx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&ctx->lock);
    
    /* Wait for interrupt */
    ret = ch375_wait_int(ctx, WAIT_INT_TIMEOUT_MS);
    if (ret != CH375_SUCCESS) {
        return CH375_TIMEOUT;
    }
    
    ret = ch375_get_status(ctx, &st);
    if (ret != CH375_SUCCESS) {
        return CH375_ERROR;
    }
    
    *status = st;
    return CH375_SUCCESS;
}

/* Context management */
void *ch375_get_priv(struct ch375_context *ctx)
{
    if (!ctx) {
        return NULL;
    }
    
    return ctx->priv;
}

int ch375_close_context(struct ch375_context *ctx)
{
    if (!ctx) {
        return CH375_PARAM_INVALID;
    }
    
    k_free(ctx);
    return CH375_SUCCESS;
}

int ch375_open_context(struct ch375_context **ctx,
                      ch375_write_cmd_fn write_cmd,
                      ch375_write_data_fn write_data,
                      ch375_read_data_fn read_data,
                      ch375_query_int_fn query_int,
                      void *priv)
{
    struct ch375_context *new_ctx;
    
    if (!ctx || !write_cmd || !write_data || !read_data || !query_int) {
        LOG_ERR("Invalid parameters");
        return CH375_PARAM_INVALID;
    }
    
    new_ctx = k_malloc(sizeof(struct ch375_context));
    if (!new_ctx) {
        LOG_ERR("Failed to allocate context");
        return CH375_ERROR;
    }
    
    memset(new_ctx, 0, sizeof(struct ch375_context));
    k_mutex_init(&new_ctx->lock);
    
    new_ctx->priv = priv;
    new_ctx->write_cmd = write_cmd;
    new_ctx->write_data = write_data;
    new_ctx->read_data = read_data;
    new_ctx->query_int = query_int;
    
    *ctx = new_ctx;
    
    LOG_DBG("CH375 context opened");
    return CH375_SUCCESS;
}