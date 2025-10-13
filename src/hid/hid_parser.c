/* HID Parser Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdint.h>

#include "hid/hid_parser.h"
#include "hid/hid_mouse.h"

LOG_MODULE_REGISTER(hid_parser, LOG_LEVEL_DBG);

/* HID Item tag long */
#define HID_ITEM_TAG_LONG 15

/* Safe unaligned reads */
static inline uint8_t read_u8(const uint8_t *p)
{
    return *p;
}
static inline uint16_t read_le16_unaligned(const uint8_t *p)
{
    uint16_t v;
    memcpy(&v, p, sizeof(v));
    return sys_le16_to_cpu(v);
}
static inline uint32_t read_le32_unaligned(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return sys_le32_to_cpu(v);
}

/* Fetch next item from HID report descriptor */
uint8_t *hid_fetch_item(uint8_t *start, uint8_t *end, struct hid_item *item)
{
    uint8_t fb;

    if (!start || !end || !item) {
        return NULL;
    }

    if (start >= end) {
        return NULL;
    }

    fb = *start++;
    item->type = (fb >> 2) & 0x03;
    item->tag = (fb >> 4) & 0x0F;

    if (item->tag == HID_ITEM_TAG_LONG) {
        /* Long format */
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

    if (item->size == 0) {
        /* No payload */
        return start;
    } else if (item->size == 1) {
        if ((end - start) < 1) {
            return NULL;
        }
        item->data.u8 = *start++;
        return start;
    } else if (item->size == 2) {
        if ((end - start) < 2) {
            return NULL;
        }
        item->data.u16 = read_le16_unaligned(start);
        start += 2;
        return start;
    } else if (item->size == 3) {
        /* size 3 means 4 bytes per HID spec */
        if ((end - start) < 4) {
            return NULL;
        }
        item->size = 4;
        item->data.u32 = read_le32_unaligned(start);
        start += 4;
        return start;
    }

    return NULL;
}


int hid_parse_report_descriptor(uint8_t *report, uint16_t len, uint8_t *type)
{
    struct hid_item item;
    uint8_t *end = report + len;
    uint8_t *cur = report;
    uint16_t current_usage_page = 0;

    if (!report || !type || len < 2) {
        return -EINVAL;
    }

    memset(&item, 0, sizeof(item));

    while (cur && cur < end) {
        cur = hid_fetch_item(cur, end, &item);
        if (!cur) {
            break;
        }

        /* Global item: Usage Page */
        if (item.format == HID_ITEM_FORMAT_SHORT &&
            item.type == HID_ITEM_TYPE_GLOBAL &&
            item.tag == HID_GLOBAL_ITEM_TAG_USAGE_PAGE) {

            if (item.size == 1) {
                current_usage_page = item.data.u8;
            } else if (item.size == 2) {
                current_usage_page = item.data.u16;
            } else {
                current_usage_page = 0;
            }

            continue;
        }

        /* Local item: Usage */
        if (item.format == HID_ITEM_FORMAT_SHORT &&
            item.type == HID_ITEM_TYPE_LOCAL &&
            item.tag == HID_LOCAL_ITEM_TAG_USAGE) {

            uint32_t usage_val = 0;
            if (item.size == 1) usage_val = item.data.u8;
            else if (item.size == 2) usage_val = item.data.u16;
            else if (item.size == 4) usage_val = item.data.u32;
            else usage_val = 0;

            /* Generic Desktop Page = 0x01 */
            if (current_usage_page == 0x01) {
                if (usage_val == 0x02) { /* Mouse */
                    *type = USBHID_TYPE_MOUSE;
                    LOG_INF("Detected HID Mouse");
                    return 0;
                } else if (usage_val == 0x06) { /* Keyboard */
                    *type = USBHID_TYPE_KEYBOARD;
                    LOG_INF("Detected HID Keyboard");
                    return 0;
                }
            }
        }
    }

    LOG_ERR("No usable usage found in descriptor");
    return -ENOTSUP;
}

