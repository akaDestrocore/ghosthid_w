/* src/ch375_usbhost.c - Zephyr-adapted from original ch375_usbhost.c */

#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <stdlib.h>

#include "bswap.h"
#include "ch375.h"
#include "ch375_usbhost.h"

LOG_MODULE_REGISTER(ch375_usbhost, LOG_LEVEL_DBG);

#define SETUP_IN(x) ((x) & 0x80)
#define EP_IN(x) ((x) & 0x80)

#define RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS 1000
#define TRANSFER_TIMEOUT 5000

#define USB_DEFAULT_ADDRESS 1
#define USB_DEFAULT_EP0_MAX_PACKSIZE 8

static inline void fill_control_setup(uint8_t *buf,
    uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    assert(buf);
    USB_ControlSetup_t *cs = (USB_ControlSetup_t *)buf;
    cs->bmRequestType = request_type;
    cs->bRequest = bRequest;
    cs->wValue = cpu_to_le16(wValue);
    cs->wIndex = cpu_to_le16(wIndex);
    cs->wLength = cpu_to_le16(wLength);
}

int ch375_hostControlTransfer(USB_Device_t *udev,
    uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
    uint8_t *data, uint16_t wLength, int *actual_length, uint32_t timeout)
{
    CH375_Context_t *ctx;
    uint8_t setup_buf[CONTROL_SETUP_SIZE];
    int residue_len = wLength;
    uint8_t tog = 0;
    int offset = 0;
    uint8_t status = 0;
    int ret;

    if (udev == NULL || udev->context == NULL) {
        LOG_ERR("invalid udev/context");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }
    if (data == NULL && wLength != 0) {
        LOG_ERR("param data/wLength invalid");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }
    ctx = udev->context;

    ret = ch375_setRetry(ctx, ch375_setRetry_TIMES_INFINITY);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set ch375 retry infinity failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    /* SETUP */
    LOG_DBG("send SETUP");
    fill_control_setup(setup_buf, request_type, bRequest, wValue, wIndex, wLength);
    ret = ch375_writeBlockData(ctx, setup_buf, CONTROL_SETUP_SIZE);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("write control setup failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    ret = ch375_sendToken(ctx, 0, tog, USB_PID_SETUP, &status);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("send token(SETUP) failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    if (status != CH375_USB_INT_SUCCESS) {
        LOG_ERR("send token(SETUP) failed, status=0x%02X", status);
        goto status_error;
    }
    tog ^= 1;
    LOG_DBG("send token(SETUP) done");

    /* DATA stage */
    while (residue_len) {
        uint8_t len = (residue_len > udev->ep0_maxpack) ? udev->ep0_maxpack : residue_len;
        uint8_t actual_len = 0;

        if (SETUP_IN(request_type)) {
            /* IN */
            ret = ch375_sendToken(ctx, 0, tog, USB_PID_IN, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("send token(IN) failed, ret=%d", ret);
                return CH375_HST_ERRNO_ERROR;
            }
            if (status != CH375_USB_INT_SUCCESS) {
                LOG_ERR("send token(IN) failed, status=0x%02X", status);
                goto status_error;
            }
            tog ^= 1;

            ret = ch375_readBlockData(ctx, data + offset, len, &actual_len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("read control in data failed (residue=%d,len=%d,actual=%d) ret=%d",
                    residue_len, len, actual_len, ret);
                return CH375_HST_ERRNO_ERROR;
            }
            residue_len -= actual_len;
            offset += actual_len;
            if (actual_len < udev->ep0_maxpack) {
                /* short packet -> done */
                break;
            }
        } else {
            /* OUT */
            ret = ch375_writeBlockData(ctx, data + offset, len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("write control OUT data failed (residue=%d,len=%d) ret=%d",
                    residue_len, len, ret);
                return CH375_HST_ERRNO_ERROR;
            }
            ret = ch375_sendToken(ctx, 0, tog, USB_PID_OUT, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("send token(OUT) failed, ret=%d", ret);
                return CH375_HST_ERRNO_ERROR;
            }
            if (status != CH375_USB_INT_SUCCESS) {
                LOG_ERR("send token(OUT) failed, status=0x%02X", status);
                goto status_error;
            }
            tog ^= 1;
            residue_len -= len;
            offset += len;
        }
    }

    /* ACK stage */
    tog = 1;
    if (SETUP_IN(request_type)) {
        ret = ch375_writeBlockData(ctx, data + offset, 0);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("write control OUT 2 data failed, ret=%d", ret);
            return CH375_HST_ERRNO_ERROR;
        }
        ret = ch375_sendToken(ctx, 0, tog, USB_PID_OUT, &status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("send token(OUT 2) failed, ret=%d", ret);
            return CH375_HST_ERRNO_ERROR;
        }
        if (status != CH375_USB_INT_SUCCESS) {
            LOG_ERR("send token(OUT 2) failed, status=0x%02X", status);
            goto status_error;
        }
    } else {
        ret = ch375_sendToken(ctx, 0, tog, USB_PID_IN, &status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("send token(IN 2) failed, ret=%d", ret);
            return CH375_HST_ERRNO_ERROR;
        }
        if (status != CH375_USB_INT_SUCCESS) {
            LOG_ERR("send token(IN 2) failed, status=0x%02X", status);
            goto status_error;
        }
    }

    if (actual_length) *actual_length = offset;
    return CH375_HST_ERRNO_SUCCESS;

status_error:
    if (status == CH375_USB_INT_DISCONNECT) {
        return CH375_HST_ERRNO_DEV_DISCONNECT;
    }
    if (status == CH375_PID2STATUS(CH375_USB_PID_STALL)) {
        return CH375_HST_ERRNO_STALL;
    }
    LOG_ERR("unhandled status=0x%02X", status);
    return CH375_HST_ERRNO_ERROR;
}

static int get_ep(USB_Device_t *udev, uint8_t ep_addr, USB_Endpoint_t **ep)
{
    assert(udev);
    assert(ep_addr != 0);
    assert(ep);
    for (int i = 0; i < udev->interface_cnt; i++) {
        USB_Interface_t *interface = &udev->interface[i];
        for (int j = 0; j < interface->endpoint_cnt; j++) {
            if (interface->endpoint[j].ep_num == ep_addr) {
                *ep = &interface->endpoint[j];
                return 0;
            }
        }
    }
    return -1;
}

int ch375_hostBulkTransfer(USB_Device_t *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout)
{
    CH375_Context_t *ctx;
    USB_Endpoint_t *endpoint = NULL;
    int residue_len = length;
    int offset = 0;
    uint8_t status = 0;
    int ret;

    if (udev == NULL || udev->context == NULL) {
        LOG_ERR("invalid udev/context");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }
    if (data == NULL && length != 0) {
        LOG_ERR("param data/length invalid");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }
    ctx = udev->context;

    ret = get_ep(udev, ep, &endpoint);
    if (ret < 0) {
        LOG_ERR("invalid ep 0x%02X", ep);
        return CH375_HST_ERRNO_PARAM_INVALID;
    }

    ret = ch375_setRetry(ctx, ch375_setRetry_TIMES_ZROE);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set ch375 retry failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    while (residue_len > 0) {
        uint8_t len = (residue_len > endpoint->maxpack) ? endpoint->maxpack : residue_len;
        uint8_t actual_len = 0;

        if (EP_IN(ep)) {
            ret = ch375_sendToken(ctx, ep, endpoint->tog, USB_PID_IN, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("send token(IN) to ep(0x%02X) failed", ep);
                return CH375_HST_ERRNO_ERROR;
            }
            if (status == CH375_USB_INT_SUCCESS) {
                ret = ch375_readBlockData(ctx, data + offset, len, &actual_len);
                if (ret != CH375_SUCCESS) {
                    LOG_ERR("read IN data(len=%d) failed, ret=%d", len, ret);
                    return CH375_HST_ERRNO_ERROR;
                }
            }
        } else {
            ret = ch375_writeBlockData(ctx, data + offset, len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("write OUT data failed, len=%d ret=%d", len, ret);
                return CH375_HST_ERRNO_ERROR;
            }
            ret = ch375_sendToken(ctx, ep, endpoint->tog, USB_PID_OUT, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("send token(OUT) to ep(0x%02X) failed", ep);
                return CH375_HST_ERRNO_ERROR;
            }
        }

        LOG_DBG("offset=%d, residue=%d, len=%d, status=0x%02X",
            offset, residue_len, len, status);

        if (status == CH375_USB_INT_SUCCESS) {
            LOG_DBG("actual_len=%d", actual_len);
            endpoint->tog ^= 1;
            offset += actual_len;
            residue_len -= actual_len;
            continue;
        }

        if (status == CH375_PID2STATUS(CH375_USB_PID_NAK)) {
            if (timeout == 0) {
                return CH375_HST_ERRNO_TIMEOUT;
            }
            timeout--;
            k_msleep(1);
        } else {
            LOG_ERR("transfer failed, status=0x%02X", status);
            goto status_error;
        }
    }

    if (actual_length) *actual_length = offset;
    return CH375_HST_ERRNO_SUCCESS;

status_error:
    if (status == CH375_USB_INT_DISCONNECT) {
        return CH375_HST_ERRNO_DEV_DISCONNECT;
    }
    if (status == CH375_PID2STATUS(CH375_USB_PID_STALL)) {
        return CH375_HST_ERRNO_STALL;
    }
    LOG_ERR("unhandled status=0x%02X", status);
    return CH375_HST_ERRNO_ERROR;
}

int ch375_hostInterruptTransfer(USB_Device_t *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout)
{
    return ch375_hostBulkTransfer(udev, ep, data, length, actual_length, timeout);
}

static void parser_endpoint_descriptor(USB_Interface_t *interface, USB_EndpointDescriptor_t *desc)
{
    assert(interface);
    assert(desc);

    USB_Endpoint_t *ep = &interface->endpoint[interface->endpoint_cnt];
    ep->ep_num = desc->bEndpointAddress;
    ep->tog = 0;
    ep->attr = desc->bmAttributes;
    ep->maxpack = le16_to_cpu(desc->wMaxPacketSize);
    ep->interval = desc->bInterval;

    interface->endpoint_cnt++;
}

static void parser_interface_descriptor(USB_Device_t *udev, USB_InterfaceDescriptor_t *desc)
{
    assert(udev);
    assert(desc);

    USB_Interface_t *interface = &udev->interface[udev->interface_cnt];
    interface->interface_num = desc->bInterfaceNumber;
    interface->interface_class = desc->bInterfaceClass;
    interface->subclass = desc->bInterfaceSubClass;
    interface->protocol = desc->bInterfaceProtocol;

    udev->interface_cnt++;
}

static int parser_config_descriptor(USB_Device_t *udev)
{
    USB_Descriptor_t *desc = (USB_Descriptor_t *)udev->raw_conf_desc;
    void *raw_conf_desc_end = (uint8_t *)udev->raw_conf_desc + udev->raw_conf_desc_len;

    while ((void *)desc < raw_conf_desc_end) {
        desc = (USB_Descriptor_t *)((uint8_t *)desc + desc->bLength);

        switch (desc->bDesriptorType) {
            case USB_DT_INTERFACE:
                parser_interface_descriptor(udev, (USB_InterfaceDescriptor_t *)desc);
                break;
            case USB_DT_ENDPOINT: {
                if (udev->interface_cnt == 0) {
                    LOG_ERR("invalid config descriptor endpoint before interface");
                    return CH375_HST_ERRNO_ERROR;
                }
                uint8_t last_interface_idx = udev->interface_cnt - 1;
                parser_endpoint_descriptor(&udev->interface[last_interface_idx], (USB_EndpointDescriptor_t *)desc);
                break;
            }
            default:
                break;
        }
    }
    return CH375_HST_ERRNO_SUCCESS;
}

static int get_config_descriptor(USB_Device_t *udev, uint8_t *buf, uint16_t len)
{
    assert(udev && buf);
    int actual_len = 0;
    int ret;

    assert(len >= sizeof(USB_ConfigDescriptor_t));

    ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_IN | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_DEVICE,
        USB_REQUEST_GET_DESCRIPTOR,
        USB_DT_CONFIG << 8, 0,
        buf, len, &actual_len, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("control transfer failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    if (actual_len < len) {
        LOG_ERR("no enough data");
        return CH375_HST_ERRNO_ERROR;
    }
    return CH375_HST_ERRNO_SUCCESS;
}

static int ch375_set_dev_address(USB_Device_t *udev, uint8_t addr)
{
    assert(udev);
    assert(addr != 0);

    int ret;
    ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_OUT | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_DEVICE,
        USB_REQUEST_SET_ADDRESS,
        addr, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("control transfer failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    ret = ch375_setUSBAddr(udev->context, addr);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set ch375 usb addr(0x%02X) failed, ret=%d", addr, ret);
        return CH375_HST_ERRNO_ERROR;
    }
    return CH375_HST_ERRNO_SUCCESS;
}

static int get_device_descriptor(USB_Device_t *udev, uint8_t *buf)
{
    assert(buf);
    int actual_len = 0;
    int ret;
    USB_DeviceDescriptor_t *dev_desc = (USB_DeviceDescriptor_t *)buf;

    ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_IN | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_DEVICE,
        USB_REQUEST_GET_DESCRIPTOR,
        USB_DT_DEVICE << 8, 0,
        buf, sizeof(USB_DeviceDescriptor_t), &actual_len, TRANSFER_TIMEOUT);

    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("control transfer failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    if (actual_len < sizeof(USB_DeviceDescriptor_t)) {
        LOG_ERR("no enough data");
        return CH375_HST_ERRNO_ERROR;
    }
    if (dev_desc->bDescriptorType != USB_DT_DEVICE) {
        LOG_ERR("invalid device descriptor, got=0x%02X expected=0x%02X",
            dev_desc->bDescriptorType, USB_DT_DEVICE);
        return CH375_HST_ERRNO_ERROR;
    }
    return CH375_HST_ERRNO_SUCCESS;
}

int ch375_hostClearStall(USB_Device_t *udev, uint8_t ep)
{
    USB_Endpoint_t *endpoint = NULL;
    int ret;

    if (ep != 0) {
        ret = get_ep(udev, ep, &endpoint);
        if (ret < 0) {
            LOG_ERR("invalid ep 0x%02X", ep);
            return CH375_HST_ERRNO_PARAM_INVALID;
        }
    }

    ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_IN | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_ENDPOINT,
        USB_REQUEST_CLEAR_FEATURE,
        0, ep, NULL, 0, NULL, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("clear feature failed ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    if (endpoint) {
        endpoint->tog = 0;
    }
    return CH375_HST_ERRNO_SUCCESS;
}

int ch375_hostSetConfigration(USB_Device_t *udev, uint8_t iconfigration)
{
    int ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_OUT | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_DEVICE,
        USB_REQUEST_SET_CONFIGURATION,
        iconfigration, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("set configuration(%d) failed, ret=%d", iconfigration, ret);
        return CH375_HST_ERRNO_ERROR;
    }
    return CH375_HST_ERRNO_SUCCESS;
}

static int reset_dev(CH375_Context_t *ctx)
{
    assert(ctx);
    int ret;
    ret = ch375_setUSBMode(ctx, CH375_USB_MODE_RESET);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set usbmode RESET failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    k_msleep(20);

    ret = ch375_setUSBMode(ctx, CH375_USB_MODE_SOF);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set usbmode SOF failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }

    ret = ch375_hostWaitDeviceConnect(ctx, RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("wait device connect failed, ret=%d", ret);
        ret = ch375_setUSBMode(ctx, CH375_USB_MODE_SOF);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("set usb mode SOF failed, ret=%d", ret);
            return CH375_HST_ERRNO_ERROR;
        }
        return CH375_HST_ERRNO_DEV_DISCONNECT;
    }
    k_msleep(100);
    return CH375_HST_ERRNO_SUCCESS;
}

int ch375_hostResetDev(USB_Device_t *udev)
{
    int ret;
    uint8_t conn_status;
    uint8_t speed;
    CH375_Context_t *ctx;

    if (udev == NULL || udev->context == NULL) {
        LOG_ERR("invalid udev/context");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }

    ctx = udev->context;
    udev->ready = 0;

    ret = ch375_testConnect(ctx, &conn_status);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("test connect failed, ret=%d", ret);
        goto error;
    }
    if (conn_status == CH375_USB_INT_DISCONNECT) {
        LOG_ERR("udev init failed, device disconnected");
        goto disconnect;
    }

    ret = ch375_getDevSpeed(ctx, &speed);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("get dev speed failed, ret=%d", ret);
        goto disconnect;
    }
    if (speed == CH375_USB_SPEED_LOW) {
        udev->speed = USB_SPEED_LOW;
        LOG_INF("device speed low");
    } else {
        udev->speed = USB_SPEED_FULL;
        LOG_INF("device speed full");
    }

    ret = reset_dev(ctx);
    if (ret == CH375_HST_ERRNO_SUCCESS) {
        /* ok */
    } else if (ret == CH375_HST_ERRNO_DEV_DISCONNECT) {
        goto disconnect;
    } else {
        goto error;
    }

    if (speed == CH375_USB_SPEED_LOW) {
        ret = ch375_setDevSpeed(ctx, speed);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("set device speed failed ret=%d", ret);
            goto disconnect;
        }
    }

    k_msleep(100);

    udev->connected = 1;
    udev->ready = 1;
    return CH375_HST_ERRNO_SUCCESS;

error:
    udev->connected = 0;
    udev->ready = 0;
    return CH375_HST_ERRNO_ERROR;

disconnect:
    udev->connected = 0;
    udev->ready = 0;
    return CH375_HST_ERRNO_DEV_DISCONNECT;
}

void ch375_hostUdevClose(USB_Device_t *udev)
{
    if (!udev) return;
    if (udev->raw_conf_desc) {
        k_free(udev->raw_conf_desc);
        udev->raw_conf_desc = NULL;
    }
    memset(udev, 0, sizeof(USB_Device_t));
}

int ch375_hostUdevOpen(CH375_Context_t *context, USB_Device_t *udev)
{
    USB_ConfigDescriptor_t short_conf_desc = {0};
    uint16_t conf_total_len = 0;
    int ret;
    uint8_t ep_cnt = 0;

    if (udev == NULL) {
        LOG_ERR("param udev NULL");
        return CH375_HST_ERRNO_PARAM_INVALID;
    }

    memset(udev, 0, sizeof(USB_Device_t));
    udev->context = context;
    udev->ep0_maxpack = USB_DEFAULT_EP0_MAX_PACKSIZE;

    ret = ch375_hostResetDev(udev);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("reset device failed, ret=%d", ret);
        return ret;
    }

    LOG_INF("getting device descriptor");
    ret = get_device_descriptor(udev, (uint8_t *)&udev->raw_dev_desc);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("get device descriptor failed, ret=%d", ret);
        goto failed;
    }

    udev->ep0_maxpack = udev->raw_dev_desc.bMaxPacketSize0;
    udev->vid = le16_to_cpu(udev->raw_dev_desc.idVendor);
    udev->pid = le16_to_cpu(udev->raw_dev_desc.idProduct);

    LOG_INF("device VID:PID = %04X:%04X", udev->vid, udev->pid);
    LOG_INF("ep0 maxpack = %d", (int)udev->ep0_maxpack);

    ret = ch375_set_dev_address(udev, USB_DEFAULT_ADDRESS);
    if (ret != CH375_HST_ERRNO_SUCCESS)  {
        LOG_ERR("set device address failed, ret=%d", ret);
        goto failed;
    }

    LOG_INF("getting short config descriptor");
    ret = get_config_descriptor(udev, (uint8_t *)&short_conf_desc, sizeof(short_conf_desc));
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("get short config descriptor failed, ret=%d", ret);
        goto failed;
    }

    conf_total_len = le16_to_cpu(short_conf_desc.wTotalLength);
    udev->configuration_value = short_conf_desc.bConfigurationValue;
    udev->raw_conf_desc_len = conf_total_len;
    LOG_INF("config wTotalLength=%d", conf_total_len);

    udev->raw_conf_desc = (uint8_t *)k_malloc(conf_total_len);
    if (udev->raw_conf_desc == NULL) {
        LOG_ERR("alloc config descriptor buffer failed");
        goto failed;
    }
    memset(udev->raw_conf_desc, 0, conf_total_len);

    ret = get_config_descriptor(udev, udev->raw_conf_desc, conf_total_len);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("get full config descriptor failed, ret=%d", ret);
        goto failed;
    }

    ret = parser_config_descriptor(udev);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("parser config descriptor failed, ret=%d", ret);
        goto failed;
    }

    for (int i = 0; i < udev->interface_cnt; i++) {
        ep_cnt += udev->interface[i].endpoint_cnt;
    }
    LOG_INF("device has %d interfaces, %d endpoints", udev->interface_cnt, ep_cnt);

    ret = ch375_hostSetConfigration(udev, udev->configuration_value);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("set configuration(%d) failed, ret=%d", udev->configuration_value, ret);
        goto failed;
    }

    LOG_INF("set configuration %d success", udev->configuration_value);
    udev->connected = 1;

    return CH375_HST_ERRNO_SUCCESS;

failed:
    if (udev->raw_conf_desc != NULL) {
        k_free(udev->raw_conf_desc);
        udev->raw_conf_desc = NULL;
    }
    memset(udev, 0, sizeof(USB_Device_t));
    return CH375_HST_ERRNO_ERROR;
}

int ch375_hostWaitDeviceConnect(CH375_Context_t *context, uint32_t timeout)
{
    int ret;
    uint8_t conn_status;
    uint32_t cnt;

    for (cnt = 0; cnt <= timeout; cnt++) {
        ret = ch375_testConnect(context, &conn_status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("test connect failed, ret=%d", ret);
            return CH375_HST_ERRNO_ERROR;
        }
        if (conn_status == CH375_USB_INT_DISCONNECT) {
            continue;
        }
        return CH375_HST_ERRNO_SUCCESS;
    }

    return CH375_HST_ERRNO_TIMEOUT;
}

int ch375_hostInit(CH375_Context_t *context, uint32_t work_baudrate)
{
    int ret;

    if (work_baudrate != 9600 && work_baudrate != 115200) {
        LOG_ERR("work_baudrate(%lu) not supported", (unsigned long)work_baudrate);
        return CH375_HST_ERRNO_PARAM_INVALID;
    }

    LOG_INF("wait 50ms before init");
    k_msleep(50);

    ret = ch375_checkExist(context);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("check exist failed %d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    LOG_INF("ch375 exist check done");

    ret = ch375_setUSBMode(context, CH375_USB_MODE_SOF);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set USB mode SOF failed, ret=%d", ret);
        return CH375_HST_ERRNO_ERROR;
    }
    LOG_INF("set usb mode to Host, Auto-detection, No SOF");

    ret = ch375_setBaudrate(context, work_baudrate);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("set work baudrate(%lu) failed", (unsigned long)work_baudrate);
        return CH375_HST_ERRNO_ERROR;
    }
    LOG_INF("set work baudrate to %lu", (unsigned long)work_baudrate);
    k_msleep(5);

    return CH375_HST_ERRNO_SUCCESS;
}
