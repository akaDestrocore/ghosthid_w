/* CH375 USB Host Mode - COMPLETE FIX with correct USB request types */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>

#include "ch375/ch375.h"
#include "ch375/ch375_host.h"
#include "ch375/usb.h"

LOG_MODULE_REGISTER(ch375_host, LOG_LEVEL_DBG);

/* USB Request Type Definitions - DO NOT USE Zephyr's confusing macros */
#define USB_DIR_OUT             0x00
#define USB_DIR_IN              0x80
#define USB_TYPE_STANDARD       0x00
#define USB_TYPE_CLASS          0x20
#define USB_TYPE_VENDOR         0x40
#define USB_RECIP_DEVICE        0x00
#define USB_RECIP_INTERFACE     0x01
#define USB_RECIP_ENDPOINT      0x02

#define USB_REQ_TYPE(dir, type, recip) ((dir) | (type) | (recip))

#define SETUP_IN(x) ((x) & 0x80)
#define EP_IN(x) ((x) & 0x80)

#define RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS 1000
#define TRANSFER_TIMEOUT 5000

#define USB_DEFAULT_ADDRESS 1
#define USB_DEFAULT_EP0_MAX_PACKSIZE 8

/* Private function prototypes */
static int get_ep(struct usb_device *udev, uint8_t ep_addr, struct usb_endpoint **ep);

/* Fill control setup packet */
static inline void fill_control_setup(uint8_t *buf,
    uint8_t request_type, uint8_t bRequest, uint16_t wValue, 
    uint16_t wIndex, uint16_t wLength)
{
    struct usb_setup_packet *setup = (struct usb_setup_packet *)buf;
    setup->bmRequestType = request_type;
    setup->bRequest = bRequest;
    setup->wValue = sys_cpu_to_le16(wValue);
    setup->wIndex = sys_cpu_to_le16(wIndex);
    setup->wLength = sys_cpu_to_le16(wLength);
}

/* Fixed Control Transfer - Keep toggle at 1 for all DATA packets */

int ch375_host_control_transfer(struct usb_device *udev,
    uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
    uint8_t *data, uint16_t wLength, int *actual_length, uint32_t timeout)
{
    struct ch375_context *ctx;
    uint8_t setup_buf[CONTROL_SETUP_SIZE];
    int residue_len = wLength;
    uint8_t tog = 0;
    int offset = 0;
    uint8_t status;
    int ret;
    
    if (!udev || !udev->context) {
        LOG_ERR("Invalid device or context");
        return CH375_HOST_PARAM_INVALID;
    }
    
    if (!data && wLength != 0) {
        LOG_ERR("Invalid data/length parameters");
        return CH375_HOST_PARAM_INVALID;
    }
    
    ctx = udev->context;
    
    ret = ch375_set_retry(ctx, CH375_RETRY_TIMES_INFINITY);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set retry failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    /* SETUP stage - always uses tog=0 */
    LOG_DBG("Sending SETUP ...");
    fill_control_setup(setup_buf, request_type, bRequest, wValue, wIndex, wLength);
    
    ret = ch375_write_block_data(ctx, setup_buf, CONTROL_SETUP_SIZE);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Write SETUP packet failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    ret = ch375_send_token(ctx, 0, tog, CH375_USB_PID_SETUP, &status);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Send SETUP token failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    if (status != CH375_USB_INT_SUCCESS) {
        LOG_ERR("SETUP failed, status: 0x%02X", status);
        goto status_error;
    }
    
    tog = tog ^ 1;
    LOG_DBG("SETUP succeeded");
    
    /* DATA stage - tog is now 1 */
    while (residue_len) {
        uint8_t len = residue_len > udev->ep0_maxpack ? udev->ep0_maxpack : residue_len;
        uint8_t actual_len = 0;
        
        if (SETUP_IN(request_type)) {
            /* IN transfer */
            LOG_DBG("Sending IN token (tog=%d, remaining=%d)", tog, residue_len);
            
            ret = ch375_send_token(ctx, 0, tog, CH375_USB_PID_IN, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Send IN token failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            if (status != CH375_USB_INT_SUCCESS) {
                LOG_ERR("IN token failed, status: 0x%02X", status);
                goto status_error;
            }
            
            ret = ch375_read_block_data(ctx, data + offset, len, &actual_len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Read data failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            LOG_DBG("IN token succeeded, read %d bytes", actual_len);
            
            residue_len -= actual_len;
            offset += actual_len;
            
            // Only toggle on short packet OR when all data received
            if (actual_len < udev->ep0_maxpack || residue_len == 0) {
                LOG_DBG("Short packet or end of data, will toggle for next stage");
                tog = tog ^ 1;
                break;  // Short packet ends the DATA stage
            }
            // Don't toggle - keep same toggle for next packet
        } else {
            /* OUT transfer */
            ret = ch375_write_block_data(ctx, data + offset, len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Write data failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            ret = ch375_send_token(ctx, 0, tog, CH375_USB_PID_OUT, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Send OUT token failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            if (status != CH375_USB_INT_SUCCESS) {
                LOG_ERR("OUT token failed, status: 0x%02X", status);
                goto status_error;
            }
            
            /* Toggle AFTER success */
            tog = tog ^ 1;
            
            residue_len -= len;
            offset += len;
        }
    }
    
    /* STATUS stage - always uses tog=1 (DATA1) */
    if (SETUP_IN(request_type)) {
        /* IN control: STATUS is OUT with no data */
        LOG_DBG("Sending STATUS OUT");
        
        ret = ch375_write_block_data(ctx, NULL, 0);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Write status OUT failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        
        ret = ch375_send_token(ctx, 0, 1, CH375_USB_PID_OUT, &status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Send status OUT token failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        
        if (status != CH375_USB_INT_SUCCESS) {
            LOG_ERR("Status OUT failed: 0x%02X", status);
            goto status_error;
        }
    } else {
        /* OUT control: STATUS is IN with no data */
        LOG_DBG("Sending STATUS IN");
        
        ret = ch375_send_token(ctx, 0, 1, CH375_USB_PID_IN, &status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Send status IN token failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        
        if (status != CH375_USB_INT_SUCCESS) {
            LOG_ERR("Status IN failed: 0x%02X", status);
            goto status_error;
        }
    }
    
    LOG_DBG("Control transfer completed, transferred %d bytes", offset);
    
    if (actual_length) {
        *actual_length = offset;
    }
    
    return CH375_HOST_SUCCESS;

status_error:
    if (status == CH375_USB_INT_DISCONNECT) {
        return CH375_HOST_DEV_DISCONNECT;
    }
    if (status == CH375_PID2STATUS(CH375_USB_PID_STALL)) {
        return CH375_HOST_STALL;
    }
    LOG_ERR("Unhandled status: 0x%02X", status);
    return CH375_HOST_ERROR;
}

/* Get device descriptor - FIXED */
static int get_device_descriptor(struct usb_device *udev, uint8_t *buf)
{
    int actual_len = 0;
    int ret;
    struct usb_device_descriptor *dev_desc = (struct usb_device_descriptor *)buf;
    
    /* Request type: 0x80 = IN | STANDARD | DEVICE */
    ret = ch375_host_control_transfer(udev,
        USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_GET_DESCRIPTOR,
        USB_DESC_DEVICE << 8, 0,
        buf, sizeof(struct usb_device_descriptor), &actual_len, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Get device descriptor failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    if (actual_len < sizeof(struct usb_device_descriptor)) {
        LOG_ERR("Device descriptor too short");
        return CH375_HOST_ERROR;
    }
    
    if (dev_desc->bDescriptorType != USB_DESC_DEVICE) {
        LOG_ERR("Invalid device descriptor type: 0x%02X", dev_desc->bDescriptorType);
        return CH375_HOST_ERROR;
    }
    
    return CH375_HOST_SUCCESS;
}

/* Get configuration descriptor - FIXED */
static int get_config_descriptor(struct usb_device *udev, uint8_t *buf, uint16_t len)
{
    int actual_len = 0;
    int ret;
    
    /* Request type: 0x80 = IN | STANDARD | DEVICE */
    ret = ch375_host_control_transfer(udev,
        USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_GET_DESCRIPTOR,
        USB_DT_CONFIG << 8, 0,
        buf, len, &actual_len, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Get config descriptor failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    if (actual_len < len) {
        LOG_ERR("Config descriptor too short");
        return CH375_HOST_ERROR;
    }
    
    return CH375_HOST_SUCCESS;
}

/* Set device address - FIXED */
static int ch375_set_dev_address(struct usb_device *udev, uint8_t addr)
{
    int ret;
    
    /* Request type: 0x00 = OUT | STANDARD | DEVICE */
    ret = ch375_host_control_transfer(udev,
        USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_SET_ADDRESS,
        addr, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Set address failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    /* Tell CH375 the new address */
    ret = ch375_set_usb_addr(udev->context, addr);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set CH375 USB addr failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    return CH375_HOST_SUCCESS;
}

/* Clear endpoint stall - FIXED */
int ch375_host_clear_stall(struct usb_device *udev, uint8_t ep)
{
    struct usb_endpoint *endpoint = NULL;
    int ret;
    
    if (ep != 0) {
        ret = get_ep(udev, ep, &endpoint);
        if (ret < 0) {
            LOG_ERR("Endpoint 0x%02X not found", ep);
            return CH375_HOST_PARAM_INVALID;
        }
    }
    
    /* Request type: 0x02 = OUT | STANDARD | ENDPOINT */
    ret = ch375_host_control_transfer(udev,
        USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_ENDPOINT),
        USB_SREQ_CLEAR_FEATURE,
        0, ep, NULL, 0, NULL, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Clear feature failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    if (endpoint) {
        endpoint->tog = 0;
    }
    
    return CH375_HOST_SUCCESS;
}

/* Set configuration - FIXED */
int ch375_host_set_configuration(struct usb_device *udev, uint8_t configuration)
{
    /* Request type: 0x00 = OUT | STANDARD | DEVICE */
    int ret = ch375_host_control_transfer(udev,
        USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_SET_CONFIGURATION,
        configuration, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
        
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Set configuration failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    return CH375_HOST_SUCCESS;
}

/* Get endpoint from device */
static int get_ep(struct usb_device *udev, uint8_t ep_addr, struct usb_endpoint **ep)
{
    int i, j;
    
    if (!udev || !ep || ep_addr == 0) {
        return -1;
    }
    
    for (i = 0; i < udev->interface_cnt; i++) {
        struct usb_interface *interface = &udev->interface[i];
        for (j = 0; j < interface->endpoint_cnt; j++) {
            if (interface->endpoint[j].ep_num == ep_addr) {
                *ep = &interface->endpoint[j];
                return 0;
            }
        }
    }
    
    return -1;
}

/* Host bulk transfer */
int ch375_host_bulk_transfer(struct usb_device *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout)
{
    struct ch375_context *ctx;
    struct usb_endpoint *endpoint = NULL;
    int residue_len = length;
    int offset = 0;
    uint8_t status;
    int ret;
    
    if (!udev || !udev->context) {
        LOG_ERR("Invalid device or context");
        return CH375_HOST_PARAM_INVALID;
    }
    
    if (!data && length != 0) {
        LOG_ERR("Invalid data/length parameters");
        return CH375_HOST_PARAM_INVALID;
    }
    
    ctx = udev->context;
    
    ret = get_ep(udev, ep, &endpoint);
    if (ret < 0) {
        LOG_ERR("Endpoint 0x%02X not found", ep);
        return CH375_HOST_PARAM_INVALID;
    }
    
    ret = ch375_set_retry(ctx, CH375_RETRY_TIMES_ZERO);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set retry failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    while (residue_len > 0) {
        uint8_t len = residue_len > endpoint->maxpack ? endpoint->maxpack : residue_len;
        uint8_t actual_len = 0;
        
        if (EP_IN(ep)) {
            ret = ch375_send_token(ctx, ep, endpoint->tog, CH375_USB_PID_IN, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Send IN token failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            if (status == CH375_USB_INT_SUCCESS) {
                ret = ch375_read_block_data(ctx, data + offset, len, &actual_len);
                if (ret != CH375_SUCCESS) {
                    LOG_ERR("Read data failed: %d", ret);
                    return CH375_HOST_ERROR;
                }
            }
        } else {
            ret = ch375_write_block_data(ctx, data + offset, len);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Write data failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            ret = ch375_send_token(ctx, ep, endpoint->tog, CH375_USB_PID_OUT, &status);
            if (ret != CH375_SUCCESS) {
                LOG_ERR("Send OUT token failed: %d", ret);
                return CH375_HOST_ERROR;
            }
            
            if (status == CH375_USB_INT_SUCCESS) {
                actual_len = len;
            }
        }
        
        if (status == CH375_USB_INT_SUCCESS) {
            endpoint->tog = endpoint->tog ^ 1;
            offset += actual_len;
            residue_len -= actual_len;
            continue;
        }
        
        if (status == CH375_PID2STATUS(CH375_USB_PID_NAK)) {
            if (timeout == 0) {
                return CH375_HOST_TIMEOUT;
            }
            timeout--;
            k_msleep(1);
        } else {
            LOG_ERR("Transfer failed, status: 0x%02X", status);
            goto status_error;
        }
    }
    
    if (actual_length) {
        *actual_length = offset;
    }
    
    return CH375_HOST_SUCCESS;

status_error:
    if (status == CH375_USB_INT_DISCONNECT) {
        return CH375_HOST_DEV_DISCONNECT;
    }
    if (status == CH375_PID2STATUS(CH375_USB_PID_STALL)) {
        return CH375_HOST_STALL;
    }
    LOG_ERR("Unhandled status: 0x%02X", status);
    return CH375_HOST_ERROR;
}

/* Host interrupt transfer (same as bulk for CH375) */
int ch375_host_interrupt_transfer(struct usb_device *udev,
    uint8_t ep, uint8_t *data, int length,
    int *actual_length, uint32_t timeout)
{
    return ch375_host_bulk_transfer(udev, ep, data, length, actual_length, timeout);
}

/* Parse endpoint descriptor */
static void parse_endpoint_descriptor(struct usb_interface *interface,
                                     struct usb_ep_descriptor *desc)
{
    struct usb_endpoint *ep = &interface->endpoint[interface->endpoint_cnt];
    
    ep->ep_num = desc->bEndpointAddress;
    ep->tog = 0;
    ep->attr = desc->bmAttributes;
    ep->maxpack = sys_le16_to_cpu(desc->wMaxPacketSize);
    ep->interval = desc->bInterval;
    
    interface->endpoint_cnt++;
}

/* Parse interface descriptor */
static void parse_interface_descriptor(struct usb_device *udev,
                                      struct usb_if_descriptor *desc)
{
    struct usb_interface *interface = &udev->interface[udev->interface_cnt];
    
    interface->interface_num = desc->bInterfaceNumber;
    interface->interface_class = desc->bInterfaceClass;
    interface->subclass = desc->bInterfaceSubClass;
    interface->protocol = desc->bInterfaceProtocol;
    
    udev->interface_cnt++;
}

/* Parse configuration descriptor */
static int parse_config_descriptor(struct usb_device *udev)
{
    struct usb_desc_header *desc = (struct usb_desc_header *)udev->raw_conf_desc;
    void *raw_conf_desc_end = (uint8_t *)udev->raw_conf_desc + udev->raw_conf_desc_len;
    
    while ((void *)desc < raw_conf_desc_end) {
        desc = (struct usb_desc_header *)((uint8_t *)desc + desc->bLength);
        
        switch (desc->bDescriptorType) {
        case USB_DESC_INTERFACE:
            parse_interface_descriptor(udev, (struct usb_if_descriptor *)desc);
            break;
            
        case USB_DESC_ENDPOINT:
            if (udev->interface_cnt == 0) {
                LOG_ERR("Endpoint descriptor before interface descriptor");
                return CH375_HOST_ERROR;
            }
            parse_endpoint_descriptor(&udev->interface[udev->interface_cnt - 1],
                                    (struct usb_ep_descriptor *)desc);
            break;
            
        default:
            break;
        }
    }
    
    return CH375_HOST_SUCCESS;
}

/* Reset device */
static int reset_dev(struct ch375_context *ctx)
{
    int ret;
    
    ret = ch375_set_usb_mode(ctx, CH375_USB_MODE_RESET);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set USB reset mode failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    k_msleep(20);
    
    ret = ch375_set_usb_mode(ctx, CH375_USB_MODE_SOF);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set USB SOF mode failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    ret = ch375_host_wait_device_connect(ctx, RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Wait device reconnect failed: %d", ret);
        
        ret = ch375_set_usb_mode(ctx, CH375_USB_MODE_SOF);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Set USB SOF mode failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        return CH375_HOST_DEV_DISCONNECT;
    }
    
    k_msleep(100);
    return CH375_HOST_SUCCESS;
}

/* Reset device public function */
int ch375_host_reset_dev(struct usb_device *udev)
{
    int ret;
    uint8_t conn_status;
    uint8_t speed;
    struct ch375_context *ctx;
    
    if (!udev || !udev->context) {
        LOG_ERR("Invalid device or context");
        return CH375_HOST_PARAM_INVALID;
    }
    
    ctx = udev->context;
    udev->ready = 0;
    
    ret = ch375_test_connect(ctx, &conn_status);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Test connect failed: %d", ret);
        goto error;
    }
    
    if (conn_status == CH375_USB_INT_DISCONNECT) {
        LOG_ERR("Device disconnected");
        goto disconnect;
    }
    
    ret = ch375_get_dev_speed(ctx, &speed);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Get device speed failed: %d", ret);
        goto disconnect;
    }
    
    if (speed == CH375_USB_SPEED_LOW) {
        udev->speed = USB_SPEED_LOW;
        LOG_INF("Device speed: LOW");
    } else {
        udev->speed = USB_SPEED_FULL;
        LOG_INF("Device speed: FULL");
    }
    
    ret = reset_dev(ctx);
    if (ret == CH375_HOST_SUCCESS) {
        /* Success */
    } else if (ret == CH375_HOST_DEV_DISCONNECT) {
        goto disconnect;
    } else {
        goto error;
    }
    
    if (speed == CH375_USB_SPEED_LOW) {
        ret = ch375_set_dev_speed(ctx, speed);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Set device speed failed: %d", ret);
            goto disconnect;
        }
    }
    
    k_msleep(100);
    
    udev->connected = 1;
    udev->ready = 1;
    return CH375_HOST_SUCCESS;

error:
    udev->connected = 0;
    udev->ready = 0;
    return CH375_HOST_ERROR;

disconnect:
    udev->connected = 0;
    udev->ready = 0;
    return CH375_HOST_DEV_DISCONNECT;
}

/* Close USB device */
void ch375_host_udev_close(struct usb_device *udev)
{
    if (!udev) {
        return;
    }
    
    if (udev->raw_conf_desc) {
        k_free(udev->raw_conf_desc);
        udev->raw_conf_desc = NULL;
    }
    
    memset(udev, 0, sizeof(struct usb_device));
}

/* Open USB device */
int ch375_host_udev_open(struct ch375_context *ctx, struct usb_device *udev)
{
    struct usb_cfg_descriptor short_conf_desc = {0};
    uint16_t conf_total_len = 0;
    int ret;
    int i;
    uint8_t ep_cnt = 0;
    
    if (!udev) {
        LOG_ERR("Invalid device pointer");
        return CH375_HOST_PARAM_INVALID;
    }
    
    memset(udev, 0, sizeof(struct usb_device));
    udev->context = ctx;
    udev->ep0_maxpack = USB_DEFAULT_EP0_MAX_PACKSIZE;
    
    ret = ch375_host_reset_dev(udev);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Reset device failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Getting device descriptor");
    ret = get_device_descriptor(udev, (uint8_t *)&udev->raw_dev_desc);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Get device descriptor failed: %d", ret);
        goto failed;
    }
    
    udev->ep0_maxpack = udev->raw_dev_desc.bMaxPacketSize0;
    udev->vid = sys_le16_to_cpu(udev->raw_dev_desc.idVendor);
    udev->pid = sys_le16_to_cpu(udev->raw_dev_desc.idProduct);
    
    LOG_INF("Device VID:PID = %04X:%04X", udev->vid, udev->pid);
    LOG_INF("EP0 max packet size = %d", udev->ep0_maxpack);
    
    LOG_INF("Setting device address");
    ret = ch375_set_dev_address(udev, USB_DEFAULT_ADDRESS);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Set device address failed: %d", ret);
        goto failed;
    }
    
    LOG_INF("Getting config descriptor");
    ret = get_config_descriptor(udev, (uint8_t *)&short_conf_desc, 
                               sizeof(short_conf_desc));
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Get short config descriptor failed: %d", ret);
        goto failed;
    }
    
    conf_total_len = sys_le16_to_cpu(short_conf_desc.wTotalLength);
    udev->configuration_value = short_conf_desc.bConfigurationValue;
    udev->raw_conf_desc_len = conf_total_len;
    LOG_INF("Config total length = %d", conf_total_len);
    
    udev->raw_conf_desc = k_malloc(conf_total_len);
    if (!udev->raw_conf_desc) {
        LOG_ERR("Allocate config descriptor buffer failed");
        goto failed;
    }
    
    memset(udev->raw_conf_desc, 0, conf_total_len);
    
    ret = get_config_descriptor(udev, udev->raw_conf_desc, conf_total_len);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Get full config descriptor failed: %d", ret);
        goto failed;
    }
    
    ret = parse_config_descriptor(udev);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Parse config descriptor failed: %d", ret);
        goto failed;
    }
    
    for (i = 0; i < udev->interface_cnt; i++) {
        ep_cnt += udev->interface[i].endpoint_cnt;
    }
    LOG_INF("Device has %d interfaces, %d endpoints", udev->interface_cnt, ep_cnt);
    
    ret = ch375_host_set_configuration(udev, udev->configuration_value);
    if (ret != CH375_HOST_SUCCESS) {
        LOG_ERR("Set configuration failed: %d", ret);
        goto failed;
    }
    
    LOG_INF("Set configuration %d success", udev->configuration_value);
    udev->connected = 1;
    
    return CH375_HOST_SUCCESS;

failed:
    if (udev->raw_conf_desc) {
        k_free(udev->raw_conf_desc);
    }
    memset(udev, 0, sizeof(struct usb_device));
    return CH375_HOST_ERROR;
}

/* Wait for device connection */
int ch375_host_wait_device_connect(struct ch375_context *ctx, uint32_t timeout_ms)
{
    int ret;
    uint8_t conn_status;
    uint32_t cnt;
    
    for (cnt = 0; cnt <= timeout_ms; cnt++) {
        ret = ch375_test_connect(ctx, &conn_status);
        if (ret != CH375_SUCCESS) {
            LOG_ERR("Test connect failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        
        if (conn_status != CH375_USB_INT_DISCONNECT) {
            return CH375_HOST_SUCCESS;
        }
        
        k_msleep(1);
    }
    
    return CH375_HOST_TIMEOUT;
}

/* Initialize CH375 in host mode */
int ch375_host_init(struct ch375_context *ctx, uint32_t work_baudrate)
{
    int ret;
    
    if (work_baudrate != 9600 && work_baudrate != 115200) {
        LOG_ERR("Invalid baudrate: %u", work_baudrate);
        return CH375_HOST_PARAM_INVALID;
    }
    
    LOG_INF("Initializing CH375 host mode");
    k_msleep(50);
    
    ret = ch375_check_exist(ctx);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("CH375 check exist failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    LOG_INF("CH375 exists");
    
    ret = ch375_set_usb_mode(ctx, CH375_USB_MODE_SOF);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set USB mode failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    LOG_INF("Set USB mode to Host with SOF");
    
    ret = ch375_set_baudrate(ctx, work_baudrate);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("Set baudrate failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    LOG_INF("Set work baudrate to %u", work_baudrate);
    
    k_msleep(5);
    
    return CH375_HOST_SUCCESS;
}