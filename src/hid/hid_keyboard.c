/* HID Keyboard Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdlib.h>

#include "ch375/ch375_host.h"
#include "hid/hid_keyboard.h"
#include "hid/hid_parser.h"

LOG_MODULE_REGISTER(hid_keyboard, LOG_LEVEL_INF);

/* Standard HID keyboard report layout */
#define HID_KBD_REPORT_SIZE     8
#define HID_KBD_MODIFIER_OFFSET 0
#define HID_KBD_RESERVED_OFFSET 1
#define HID_KBD_KEYS_OFFSET     2
#define HID_KBD_MAX_KEYS        6

/* Parse HID report for keyboard */
static int parse_hid_report(struct hid_keyboard *keyboard, uint8_t *report, uint16_t len)
{
    struct hid_data_descriptor *mod = &keyboard->modifier;
    struct hid_data_descriptor *keys = &keyboard->keys;
    
    /* Standard HID keyboard report format:
     * Byte 0: Modifier keys
     * Byte 1: Reserved
     * Bytes 2-7: Key codes (up to 6 simultaneous keys)
     */
    
    keyboard->report_length = HID_KBD_REPORT_SIZE;
    
    /* Modifier keys descriptor */
    mod->physical_minimum = 0;
    mod->physical_maximum = 8;
    mod->logical_minimum = 0;
    mod->logical_maximum = 1;
    mod->size = 8;  /* 8 bits for modifiers */
    mod->count = 1;
    mod->report_buf_off = HID_KBD_MODIFIER_OFFSET;
    
    /* Regular keys descriptor */
    keys->physical_minimum = 0;
    keys->physical_maximum = 255;
    keys->logical_minimum = 0;
    keys->logical_maximum = 255;
    keys->size = 8;  /* 8 bits per key */
    keys->count = HID_KBD_MAX_KEYS;
    keys->report_buf_off = HID_KBD_KEYS_OFFSET;
    
    return 0;
}

int hid_keyboard_open(struct usbhid_device *usbhid_dev, struct hid_keyboard *keyboard)
{
    int ret;
    
    if (!keyboard || !usbhid_dev) {
        LOG_ERR("Invalid parameters");
        return USBHID_PARAM_INVALID;
    }
    
    if (usbhid_dev->hid_type != USBHID_TYPE_KEYBOARD) {
        LOG_ERR("Not a keyboard device");
        return USBHID_NOT_SUPPORT;
    }
    
    memset(keyboard, 0, sizeof(struct hid_keyboard));
    keyboard->hid_dev = usbhid_dev;
    
    ret = parse_hid_report(keyboard,
                          usbhid_dev->raw_hid_report_desc,
                          usbhid_dev->raw_hid_report_desc_len);
    if (ret < 0) {
        LOG_ERR("Failed to parse HID report");
        return USBHID_NOT_SUPPORT;
    }
    
    if (keyboard->report_length == 0) {
        LOG_ERR("Invalid report length");
        return USBHID_ERROR;
    }
    
    ret = usbhid_alloc_report_buffer(usbhid_dev, keyboard->report_length);
    if (ret != USBHID_SUCCESS) {
        LOG_ERR("Failed to allocate report buffer");
        return USBHID_ALLOC_FAILED;
    }
    
    LOG_INF("HID keyboard opened (report_len=%d)", keyboard->report_length);
    return USBHID_SUCCESS;
}

void hid_keyboard_close(struct hid_keyboard *keyboard)
{
    if (!keyboard) {
        return;
    }
    
    usbhid_free_report_buffer(keyboard->hid_dev);
    memset(keyboard, 0, sizeof(struct hid_keyboard));
}

int hid_keyboard_fetch_report(struct hid_keyboard *keyboard)
{
    if (!keyboard) {
        return USBHID_PARAM_INVALID;
    }
    
    return usbhid_fetch_report(keyboard->hid_dev);
}

int hid_keyboard_get_key(struct hid_keyboard *keyboard, uint32_t key_code,
                        uint32_t *value, uint8_t is_last)
{
    uint8_t *report_buf;
    uint8_t *keys_field;
    int ret;
    int i;
    
    if (!keyboard || !value) {
        return USBHID_PARAM_INVALID;
    }
    
    k_mutex_lock(&keyboard->hid_dev->report_lock, K_FOREVER);
    ret = usbhid_get_report_buffer(keyboard->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        k_mutex_unlock(&keyboard->hid_dev->report_lock);
        return ret;
    }
    
    keys_field = report_buf + keyboard->keys.report_buf_off;
    
    /* Search for the key code in the keys array */
    *value = 0;
    for (i = 0; i < HID_KBD_MAX_KEYS; i++) {
        if (keys_field[i] == key_code) {
            *value = 1;
            break;
        }
    }
    
    k_mutex_unlock(&keyboard->hid_dev->report_lock);
    return USBHID_SUCCESS;
}

int hid_keyboard_set_key(struct hid_keyboard *keyboard, uint32_t key_code,
                        uint32_t value, uint8_t is_last)
{
    uint8_t *report_buf;
    uint8_t *keys_field;
    int ret;
    int i;
    
    if (!keyboard) {
        return USBHID_PARAM_INVALID;
    }
    
    k_mutex_lock(&keyboard->hid_dev->report_lock, K_FOREVER);
    ret = usbhid_get_report_buffer(keyboard->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        k_mutex_unlock(&keyboard->hid_dev->report_lock);
        return ret;
    }
    
    keys_field = report_buf + keyboard->keys.report_buf_off;
    
    if (value) {
        /* Add key if not already present */
        for (i = 0; i < HID_KBD_MAX_KEYS; i++) {
            if (keys_field[i] == 0) {
                keys_field[i] = key_code;
                break;
            } else if (keys_field[i] == key_code) {
                /* Already present */
                break;
            }
        }
    } else {
        /* Remove key */
        for (i = 0; i < HID_KBD_MAX_KEYS; i++) {
            if (keys_field[i] == key_code) {
                keys_field[i] = 0;
                /* Shift remaining keys to fill gap */
                for (int j = i; j < HID_KBD_MAX_KEYS - 1; j++) {
                    keys_field[j] = keys_field[j + 1];
                }
                keys_field[HID_KBD_MAX_KEYS - 1] = 0;
                break;
            }
        }
    }
    
    k_mutex_unlock(&keyboard->hid_dev->report_lock);
    return USBHID_SUCCESS;
}

int hid_keyboard_get_modifier(struct hid_keyboard *keyboard, uint32_t modifier,
                             uint32_t *value, uint8_t is_last)
{
    uint8_t *report_buf;
    uint8_t *modifier_field;
    int ret;
    
    if (!keyboard || !value) {
        return USBHID_PARAM_INVALID;
    }
    
    /* Modifier should be a bit position (0-7) */
    if (modifier > 7) {
        LOG_ERR("Invalid modifier: %d", modifier);
        return USBHID_PARAM_INVALID;
    }
    
    k_mutex_lock(&keyboard->hid_dev->report_lock, K_FOREVER);
    ret = usbhid_get_report_buffer(keyboard->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        k_mutex_unlock(&keyboard->hid_dev->report_lock);
        return ret;
    }
    
    modifier_field = report_buf + keyboard->modifier.report_buf_off;
    *value = (*modifier_field & (1 << (modifier & 0x07))) ? 1 : 0;
    
    k_mutex_unlock(&keyboard->hid_dev->report_lock);
    return USBHID_SUCCESS;
}

int hid_keyboard_set_modifier(struct hid_keyboard *keyboard, uint32_t modifier,
                             uint32_t value, uint8_t is_last)
{
    uint8_t *report_buf;
    uint8_t *modifier_field;
    int ret;
    
    if (!keyboard) {
        return USBHID_PARAM_INVALID;
    }
    
    /* Modifier should be a bit position (0-7) */
    if (modifier > 7) {
        LOG_ERR("Invalid modifier: %d", modifier);
        return USBHID_PARAM_INVALID;
    }
    
    k_mutex_lock(&keyboard->hid_dev->report_lock, K_FOREVER);
    ret = usbhid_get_report_buffer(keyboard->hid_dev, &report_buf, NULL, is_last);
    if (ret != USBHID_SUCCESS) {
        k_mutex_unlock(&keyboard->hid_dev->report_lock);
        return ret;
    }
    
    modifier_field = report_buf + keyboard->modifier.report_buf_off;
    
    if (value) {
        *modifier_field |= (1 << (modifier & 0x07));
    } else {
        *modifier_field &= ~(1 << (modifier & 0x07));
    }
    
    k_mutex_unlock(&keyboard->hid_dev->report_lock);
    return USBHID_SUCCESS;
}