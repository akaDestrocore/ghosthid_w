/* HID Parser and Common Definitions for Zephyr */

#ifndef HID_PARSER_H
#define HID_PARSER_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* HID Class Codes */
#define USB_CLASS_HID 0x03

/* HID Descriptor Types */
#define USB_DESC_HID        0x21
#define USB_DESC_HID_REPORT 0x22
#define USB_DESC_HID_PHYSICAL 0x23

/* HID Report Types */
#define HID_REPORT_TYPE_INPUT   0x01
#define HID_REPORT_TYPE_OUTPUT  0x02
#define HID_REPORT_TYPE_FEATURE 0x03

/* HID Request Codes */
#define HID_GET_REPORT    0x01
#define HID_GET_IDLE      0x02
#define HID_GET_PROTOCOL  0x03
#define HID_SET_REPORT    0x09
#define HID_SET_IDLE      0x0A
#define HID_SET_PROTOCOL  0x0B

/* HID Report Item Format */
#define HID_ITEM_FORMAT_SHORT 0
#define HID_ITEM_FORMAT_LONG  1

/* HID Report Item Type */
#define HID_ITEM_TYPE_MAIN     0
#define HID_ITEM_TYPE_GLOBAL   1
#define HID_ITEM_TYPE_LOCAL    2
#define HID_ITEM_TYPE_RESERVED 3

/* HID Main Item Tags */
#define HID_MAIN_ITEM_TAG_INPUT            8
#define HID_MAIN_ITEM_TAG_OUTPUT           9
#define HID_MAIN_ITEM_TAG_FEATURE          11
#define HID_MAIN_ITEM_TAG_BEGIN_COLLECTION 10
#define HID_MAIN_ITEM_TAG_END_COLLECTION   12

/* HID Global Item Tags */
#define HID_GLOBAL_ITEM_TAG_USAGE_PAGE       0
#define HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM  1
#define HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM  2
#define HID_GLOBAL_ITEM_TAG_PHYSICAL_MINIMUM 3
#define HID_GLOBAL_ITEM_TAG_PHYSICAL_MAXIMUM 4
#define HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT    5
#define HID_GLOBAL_ITEM_TAG_UNIT             6
#define HID_GLOBAL_ITEM_TAG_REPORT_SIZE      7
#define HID_GLOBAL_ITEM_TAG_REPORT_ID        8
#define HID_GLOBAL_ITEM_TAG_REPORT_COUNT     9
#define HID_GLOBAL_ITEM_TAG_PUSH             10
#define HID_GLOBAL_ITEM_TAG_POP              11

/* HID Local Item Tags */
#define HID_LOCAL_ITEM_TAG_USAGE           0
#define HID_LOCAL_ITEM_TAG_USAGE_MINIMUM   1
#define HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM   2

/* HID Usage Pages */
#define HID_USAGE_PAGE          0xffff0000
#define HID_UP_UNDEFINED        0x00000000
#define HID_UP_GENDESK          0x00010000
#define HID_UP_KEYBOARD         0x00070000
#define HID_UP_LED              0x00080000
#define HID_UP_BUTTON           0x00090000

/* HID Usage IDs */
#define HID_USAGE               0x0000ffff
#define HID_GD_POINTER          0x00010001
#define HID_GD_MOUSE            0x00010002
#define HID_GD_KEYBOARD         0x00010006
#define HID_GD_X                0x00010030
#define HID_GD_Y                0x00010031
#define HID_GD_Z                0x00010032
#define HID_GD_WHEEL            0x00010038

/* HID Item Structure */
struct hid_item {
    uint8_t format;
    uint8_t size;
    uint8_t type;
    uint8_t tag;
    union {
        uint8_t u8;
        int8_t s8;
        uint16_t u16;
        int16_t s16;
        uint32_t u32;
        int32_t s32;
        uint8_t *longdata;
    } data;
};

/* HID Data Descriptor */
struct hid_data_descriptor {
    int32_t physical_minimum;
    int32_t physical_maximum;
    int32_t logical_minimum;
    int32_t logical_maximum;
    uint32_t size;  /* in bits */
    uint32_t count;
    uint32_t report_buf_off;  /* in bytes */
};

/* Function prototypes */
uint8_t *hid_fetch_item(uint8_t *start, uint8_t *end, struct hid_item *item);
int hid_parse_report_descriptor(uint8_t *report, uint16_t len, uint8_t *type);

#endif /* HID_PARSER_H */