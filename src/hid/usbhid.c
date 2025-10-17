/* src/hid/usbhid.c - Zephyr-adapted usbhid.c */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <assert.h>

#include "bswap.h"
#include "ch375_usbhost.h"
#include "hid/usbhid.h"

LOG_MODULE_REGISTER(usbhid, LOG_LEVEL_DBG);

#define TRANSFER_TIMEOUT 0

#define HID_GET_REPORT                0x01
#define HID_GET_IDLE                  0x02
#define HID_GET_PROTOCOL              0x03
#define HID_SET_REPORT                0x09
#define HID_SET_IDLE                  0x0A
#define HID_SET_PROTOCOL              0x0B
#define HID_REPORT_TYPE_INPUT         0x01
#define HID_REPORT_TYPE_OUTPUT        0x02
#define HID_REPORT_TYPE_FEATURE       0x03

void usbhid_free_report_buffer(USBHID_Device_t *dev)
{
    if (dev == NULL) return;
    if (dev->report_buffer) {
        k_free(dev->report_buffer);
        dev->report_buffer = NULL;
    }
    dev->report_length = 0;
    dev->report_buffer_length = 0;
    dev->report_buffer_last_offset = 0;
}

int usbhid_alloc_report_buffer(USBHID_Device_t *dev, uint32_t length)
{
    uint8_t *buf;
    uint32_t buf_len;
    if (dev == NULL) {
        LOG_ERR("dev NULL");
        return USBHID_ERRNO_PARAM_INVALID;
    }
    if (dev->report_buffer) {
        LOG_ERR("last report buffer not free");
        assert(0);
        return USBHID_ERRNO_ERROR;
    }
    buf_len = length * 2;
    buf = (uint8_t *)k_malloc(buf_len);
    if (buf == NULL) {
        LOG_ERR("allocate report buffer failed");
        return USBHID_ERRNO_ERROR;
    }
    memset(buf, 0, buf_len);

    dev->report_length = length;
    dev->report_buffer = buf;
    dev->report_buffer_length = buf_len;
    dev->report_buffer_last_offset = 0;
    return USBHID_ERRNO_SUCCESS;
}

static int usbhid_read(USBHID_Device_t *dev, uint8_t *buffer, int length, int *actual_len)
{
    assert(dev);
    assert(buffer);
    USB_Device_t *udev = dev->udev;
    int ret;

    ret = ch375_hostInterruptTransfer(udev, dev->ep_in,
        buffer, length, actual_len, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        if (ret == CH375_HST_ERRNO_DEV_DISCONNECT) {
            return USBHID_ERRNO_NO_DEV;
        }
        return USBHID_ERRNO_IO_ERROR;
    }
    return USBHID_ERRNO_SUCCESS;
}

static inline uint8_t *get_report_buffer(USBHID_Device_t *dev, uint8_t is_last)
{
    assert(dev);
    if (dev->report_buffer == NULL) {
        return NULL;
    }
    if (is_last) {
        return dev->report_buffer + dev->report_buffer_last_offset;
    } else {
        uint32_t offset = dev->report_buffer_last_offset ? 0 : dev->report_length;
        return dev->report_buffer + offset;
    }
}

int usbhid_get_report_buffer(USBHID_Device_t *dev, uint8_t **buffer, uint32_t *length, uint8_t is_last)
{
    uint8_t *buf;
    if (dev == NULL || buffer == NULL) {
        LOG_ERR("invalid params");
        return USBHID_ERRNO_PARAM_INVALID;
    }
    buf = get_report_buffer(dev, is_last);
    if (buf == NULL) {
        LOG_ERR("report buffer not allocated");
        return USBHID_ERRNO_BUFFER_NOT_ALLOC;
    }
    *buffer = buf;
    if (length) *length = dev->report_length;
    return USBHID_ERRNO_SUCCESS;
}

int usbhid_fetch_report(USBHID_Device_t *dev)
{
    uint8_t *last_report_buffer;
    int actual_len = 0;
    int ret;

    last_report_buffer = get_report_buffer(dev, 1);
    if (last_report_buffer == NULL) {
        LOG_ERR("report buffer not allocated");
        return USBHID_ERRNO_BUFFER_NOT_ALLOC;
    }

    ret = usbhid_read(dev, last_report_buffer, dev->report_length, &actual_len);
    if (ret != USBHID_ERRNO_SUCCESS) {
        return ret;
    }

    if (dev->report_buffer_last_offset) {
        dev->report_buffer_last_offset = 0;
    } else {
        dev->report_buffer_last_offset = dev->report_length;
    }
    return USBHID_ERRNO_SUCCESS;
}

/* Simple parser implemented same as original: decide whether mouse or keyboard */
static int parser_hid_report(uint8_t *report, uint16_t len, uint8_t *hid_type)
{
    HID_Item_t item = {0};
    uint8_t *end = report + len;
    uint8_t *cur = report;

    cur = hid_fetch_item(cur, end, &item);
    if (cur == NULL) {
        LOG_ERR("get item failed");
        return -1;
    }
    if (item.size != 1) {
        LOG_ERR("first item size invalid");
        return -1;
    }
    if (!(item.format == HID_ITEM_FORMAT_SHORT &&
        item.type == HID_ITEM_TYPE_GLOBAL &&
        item.tag == HID_GLOBAL_ITEM_TAG_USAGE_PAGE &&
        (item.data.u8 << 16) == HID_UP_GENDESK)) {
        LOG_ERR("first item not generic desktop");
        return -1;
    }
    cur = hid_fetch_item(cur, end, &item);
    if (cur == NULL) {
        LOG_ERR("get item failed");
        return -1;
    }
    if (item.size != 1) {
        LOG_ERR("usage item size invalid");
        return -1;
    }
    if (!(item.format == HID_ITEM_FORMAT_SHORT &&
        item.type == HID_ITEM_TYPE_LOCAL &&
        item.tag == HID_GLOBAL_ITEM_TAG_USAGE_PAGE)) {
        LOG_ERR("Usage not found");
        return -1;
    }
    if (item.data.u8 == (HID_GD_MOUSE & 0xF)) {
        *hid_type = USBHID_TYPE_MOUSE;
        return 0;
    } else if (item.data.u8 == (HID_GD_KEYBOARD & 0xF)) {
        *hid_type = USBHID_TYPE_KEYBOARD;
        return 0;
    } else {
        LOG_ERR("unsupported Usage 0x%02X", item.data.u8);
        return -1;
    }
}

static int hid_get_class_descriptor(USB_Device_t *udev, uint8_t interface_num,
    uint8_t type, uint8_t *buf, uint16_t len)
{
    assert(udev);
    uint8_t retries = 4;
    int actual_len = 0;
    int ret;

    do {
        ret = ch375_hostControlTransfer(udev,
            USB_ENDPOINT_IN | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_INTERFACE,
            USB_REQUEST_GET_DESCRIPTOR,
            type << 8,
            interface_num,
            buf, len, &actual_len, TRANSFER_TIMEOUT);
        if (ret != CH375_HST_ERRNO_SUCCESS) {
            LOG_ERR("get class descriptor(type=0x%02X) failed ret=%d", type, ret);
        }
        retries--;
    } while (actual_len < len && retries);

    if (actual_len < len) {
        LOG_ERR("not enough data actual=%d expected=%d", actual_len, len);
        return USBHID_ERRNO_ERROR;
    }
    return USBHID_ERRNO_SUCCESS;
}

static int get_ep_in(USB_Device_t *udev, uint8_t interface_num, uint8_t *ep)
{
    assert(udev);
    assert(ep);
    assert(interface_num < udev->interface_cnt);
    USB_Interface_t *interface = &udev->interface[interface_num];

    if (interface->endpoint_cnt < 1) {
        LOG_ERR("interface %d has no endpoint", interface_num);
        return -1;
    }
    *ep = interface->endpoint[0].ep_num;
    return 0;
}

static void set_idle(USB_Device_t *udev, uint8_t interface_num,
    uint8_t duration, uint8_t report_id)
{
    assert(udev);
    int ret;

    ret = ch375_hostControlTransfer(udev,
        USB_ENDPOINT_OUT | USB_REQUEST_TYPE_CLASS | USB_RECIPIENT_INTERFACE,
        HID_SET_IDLE,
        (duration << 8) | report_id,
        interface_num,
        NULL, 0, NULL, TRANSFER_TIMEOUT);
    if (ret != CH375_HST_ERRNO_SUCCESS) {
        LOG_ERR("set idle failed");
    }
}

static int set_report(USB_Device_t *udev, uint8_t interface_num,
    uint8_t report_type, uint8_t report_id)
{
    assert(udev);
    uint8_t retries = 4;
    int ret;
    int actual_len = 0;
    uint8_t data_fragment = 0x01;
    do {
        ret = ch375_hostControlTransfer(udev,
            USB_ENDPOINT_OUT | USB_REQUEST_TYPE_CLASS | USB_RECIPIENT_INTERFACE,
            HID_SET_REPORT,
            (report_type << 8) | report_id,
            interface_num,
            &data_fragment, sizeof(data_fragment), &actual_len, TRANSFER_TIMEOUT);
        if (ret != CH375_HST_ERRNO_SUCCESS) {
            LOG_ERR("set report failed, ret=%d", ret);
        }
        retries--;
    } while (actual_len != sizeof(data_fragment) && retries);
    return ret;
}

static int get_hid_descriptor(USB_Device_t *udev, uint8_t interface_num, HID_Descriptor_t **hid_desc)
{
    USB_Descriptor_t *desc = (USB_Descriptor_t *)udev->raw_conf_desc;
    void *raw_conf_desc_end = (uint8_t *)udev->raw_conf_desc + udev->raw_conf_desc_len;
    uint8_t current_interface_num = 0;

    while ((void *)desc < raw_conf_desc_end) {
        desc = (USB_Descriptor_t *)((uint8_t *)desc + desc->bLength);

        switch (desc->bDesriptorType) {
            case USB_DT_INTERFACE:
                current_interface_num = ((USB_InterfaceDescriptor_t *)desc)->bInterfaceNumber;
                break;
            case USB_DT_HID:
                if (current_interface_num == interface_num) {
                    *hid_desc = (HID_Descriptor_t *)desc;
                    return 0;
                }
                break;
            default:
                break;
        }
    }
    return -1;
}

int usbhid_open(USB_Device_t *udev, uint8_t interface_num, USBHID_Device_t *dev)
{
    HID_Descriptor_t *desc = NULL;
    uint8_t *raw_hid_report_desc = NULL;
    uint16_t raw_hid_report_desc_len = 0;
    uint8_t hid_type;
    uint8_t ep_in;
    int ret;
    int result = USBHID_ERRNO_ERROR;

    ret = get_hid_descriptor(udev, interface_num, &desc);
    if (ret < 0) {
        LOG_ERR("can't find HID descriptor in device %04X:%04X interface %d", udev->vid, udev->pid, interface_num);
        return USBHID_ERRNO_NOT_HID_DEV;
    }

    LOG_INF("HID descriptor found, version=0x%04X country=0x%02X",
        le16_to_cpu(desc->bcdHID), desc->bCountryCode);

    if (desc->bNumDescriptors > 1) {
        LOG_ERR("bNumDescriptors=%d not supported", desc->bNumDescriptors);
        return USBHID_ERRNO_NOT_SUPPORT;
    }

    memset(dev, 0, sizeof(USBHID_Device_t));

    set_idle(udev, interface_num, 0, 0);
    LOG_INF("set idle done");

    raw_hid_report_desc_len = le16_to_cpu(desc->wClassDescriptorLength);
    raw_hid_report_desc = (uint8_t *)k_malloc(raw_hid_report_desc_len);
    if (raw_hid_report_desc == NULL) {
        LOG_ERR("allocate raw HID report buffer failed");
        return USBHID_ERRNO_ERROR;
    }
    memset(raw_hid_report_desc, 0, raw_hid_report_desc_len);

    ret = get_ep_in(udev, interface_num, &ep_in);
    if (ret < 0) {
        LOG_ERR("get endpoint in failed");
        result = USBHID_ERRNO_NOT_SUPPORT;
        goto failed;
    }

    ret = hid_get_class_descriptor(udev, interface_num, USB_DT_REPORT,
        raw_hid_report_desc, raw_hid_report_desc_len);
    if (ret != USBHID_ERRNO_SUCCESS) {
        LOG_ERR("get HID REPORT failed, ret=%d", ret);
        result = USBHID_ERRNO_IO_ERROR;
        goto failed;
    }
    LOG_INF("get HID REPORT done (len=%d)", raw_hid_report_desc_len);

    ret = parser_hid_report(raw_hid_report_desc, raw_hid_report_desc_len, &hid_type);
    if (ret < 0) {
        LOG_ERR("parser HID report failed");
        result = USBHID_ERRNO_NOT_SUPPORT;
        goto failed;
    }

    if (hid_type == USBHID_TYPE_KEYBOARD) {
        ret = set_report(udev, interface_num, HID_REPORT_TYPE_OUTPUT, 0);
        if (ret != USBHID_ERRNO_SUCCESS) {
            LOG_ERR("set report failed");
            result = USBHID_ERRNO_IO_ERROR;
            goto failed;
        }
        LOG_INF("set report success for keyboard");
    }

    dev->udev = udev;
    dev->interface_num = interface_num;
    dev->ep_in = ep_in;
    dev->raw_hid_report_desc = raw_hid_report_desc;
    dev->raw_hid_report_desc_len = raw_hid_report_desc_len;
    dev->hid_desc = desc;
    dev->hid_type = hid_type;

    return USBHID_ERRNO_SUCCESS;

failed:
    if (raw_hid_report_desc) {
        k_free(raw_hid_report_desc);
        raw_hid_report_desc = NULL;
    }
    return result;
}

void usbhid_close(USBHID_Device_t *dev)
{
    if (dev == NULL) return;
    if (dev->raw_hid_report_desc) {
        k_free(dev->raw_hid_report_desc);
        dev->raw_hid_report_desc = NULL;
    }
    memset(dev, 0, sizeof(USBHID_Device_t));
}
