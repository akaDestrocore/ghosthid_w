/* HID Parser Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "hid/hid_parser.h"
#include "hid/hid_mouse.h"

LOG_MODULE_REGISTER(hid_parser, LOG_LEVEL_DBG);

/* HID Item tag long */
#define HID_ITEM_TAG_LONG 15

/* Safe hid_fetch_item: use memcpy for multi-byte fields and advance by bytes */
uint8_t *hid_fetch_item(uint8_t *start, uint8_t *end, struct hid_item *item)
{
    uint8_t fb;

    if (!start || !end || !item) {
        return NULL;
    }

    if ((end - start) <= 0) {
        return NULL;
    }

    fb = *start++;
    item->type = (fb >> 2) & 0x03;
    item->tag = (fb >> 4) & 0x0F;

    if (item->tag == HID_ITEM_TAG_LONG) {
        /* Long format requires at least two bytes (size + tag) */
        if ((end - start) < 2) {
            return NULL;
        }

        item->format = HID_ITEM_FORMAT_LONG;
        item->size = *start++;
        item->tag = *start++;

        if ((end - start) < item->size) {
            return NULL;
        }

        item->data.longdata = start;
        start += item->size;
        return start;
    }

    /* Short format */
    item->format = HID_ITEM_FORMAT_SHORT;
    item->size = fb & 0x03;

    switch (item->size) {
    case 0:
        /* No data bytes */
        item->data.u32 = 0;
        return start;

    case 1:
        if ((end - start) < 1) {
            return NULL;
        }
        item->data.u8 = *start++;
        return start;

    case 2: {
        uint16_t val16;
        if ((end - start) < 2) {
            return NULL;
        }
        memcpy(&val16, start, 2);
        item->data.u16 = sys_le16_to_cpu(val16);
        start += 2;
        return start;
    }

    case 3:
        /* 3 means 4 bytes in HID short format (special). Read as 32-bit. */
        if ((end - start) < 4) {
            return NULL;
        }
        {
            uint32_t val32;
            memcpy(&val32, start, 4);
            item->data.u32 = sys_le32_to_cpu(val32);
        }
        start += 4;
        /* adjust recorded size to 4 so callers can rely on data.u32 when needed */
        item->size = 4;
        return start;
    }

    /* unreachable */
    return NULL;
}


/* Simple HID report parser to determine device type */
int hid_parse_report_descriptor(uint8_t *report, uint16_t len, uint8_t *type)
{
    struct hid_item item = {0};
    uint8_t *end = report + len;
    uint8_t *cur = report;
    
    if (!report || !type || len < 2) {
        return -EINVAL;
    }
    
    /* First item should be Usage Page (Generic Desktop) */
    cur = hid_fetch_item(cur, end, &item);
    if (!cur) {
        LOG_ERR("Failed to fetch first item");
        return -EINVAL;
    }
    
    if (item.size != 1) {
        LOG_ERR("First item size invalid: %d", item.size);
        return -EINVAL;
    }
    
    if (!(item.format == HID_ITEM_FORMAT_SHORT &&
          item.type == HID_ITEM_TYPE_GLOBAL &&
          item.tag == HID_GLOBAL_ITEM_TAG_USAGE_PAGE &&
          (item.data.u8 << 16) == HID_UP_GENDESK)) {
        LOG_ERR("First item is not Generic Desktop Controls");
        return -EINVAL;
    }
    
    /* Next item should be Usage (Mouse/Keyboard) */
    cur = hid_fetch_item(cur, end, &item);
    if (!cur) {
        LOG_ERR("Failed to fetch second item");
        return -EINVAL;
    }
    
    if (item.size != 1) {
        LOG_ERR("Second item size invalid: %d", item.size);
        return -EINVAL;
    }
    
    if (!(item.format == HID_ITEM_FORMAT_SHORT &&
          item.type == HID_ITEM_TYPE_LOCAL &&
          item.tag == HID_LOCAL_ITEM_TAG_USAGE)) {
        LOG_ERR("Usage not found");
        return -EINVAL;
    }
    
    /* Determine device type from usage */
    if (item.data.u8 == (HID_GD_MOUSE & 0x0F)) {
        *type = USBHID_TYPE_MOUSE;
        LOG_INF("Detected HID Mouse");
        return 0;
    } else if (item.data.u8 == (HID_GD_KEYBOARD & 0x0F)) {
        *type = USBHID_TYPE_KEYBOARD;
        LOG_INF("Detected HID Keyboard");
        return 0;
    } else {
        LOG_ERR("Unknown HID Usage: 0x%02X", item.data.u8);
        return -ENOTSUP;
    }
    
    return 0;
}