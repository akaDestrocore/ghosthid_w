/* HID Keyboard Driver Header for Zephyr */

#ifndef HID_KEYBOARD_H
#define HID_KEYBOARD_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include "hid_mouse.h"  /* For shared USBHID definitions */

/* HID Keyboard Key Codes */
#define HID_KBD_LETTER(x) (((x) >= 'a' && (x) <= 'z') ? ((x) - 'a' + 4) : \
                          ((x) >= 'A' && (x) <= 'Z') ? ((x) - 'A' + 4) : 0)
#define HID_KBD_NUMBER(x) (((x) >= '1' && (x) <= '9') ? ((x) - '1' + 30) : \
                          ((x) == '0') ? 39 : 0)

/* Special Keys */
#define HID_KBD_ENTER       0x28
#define HID_KBD_ESCAPE      0x29
#define HID_KBD_BACKSPACE   0x2A
#define HID_KBD_TAB         0x2B
#define HID_KBD_SPACE       0x2C
#define HID_KBD_MINUS       0x2D
#define HID_KBD_EQUAL       0x2E
#define HID_KBD_LEFT_BRACE  0x2F
#define HID_KBD_RIGHT_BRACE 0x30
#define HID_KBD_BACKSLASH   0x31
#define HID_KBD_SEMICOLON   0x33
#define HID_KBD_APOSTROPHE  0x34
#define HID_KBD_GRAVE       0x35
#define HID_KBD_COMMA       0x36
#define HID_KBD_DOT         0x37
#define HID_KBD_SLASH       0x38
#define HID_KBD_CAPS_LOCK   0x39

/* Function Keys */
#define HID_KBD_F1          0x3A
#define HID_KBD_F2          0x3B
#define HID_KBD_F3          0x3C
#define HID_KBD_F4          0x3D
#define HID_KBD_F5          0x3E
#define HID_KBD_F6          0x3F
#define HID_KBD_F7          0x40
#define HID_KBD_F8          0x41
#define HID_KBD_F9          0x42
#define HID_KBD_F10         0x43
#define HID_KBD_F11         0x44
#define HID_KBD_F12         0x45

/* Control Keys */
#define HID_KBD_LEFT_CTRL   0xE0
#define HID_KBD_LEFT_SHIFT  0xE1
#define HID_KBD_LEFT_ALT    0xE2
#define HID_KBD_LEFT_META   0xE3
#define HID_KBD_RIGHT_CTRL  0xE4
#define HID_KBD_RIGHT_SHIFT 0xE5
#define HID_KBD_RIGHT_ALT   0xE6
#define HID_KBD_RIGHT_META  0xE7

/* HID Keyboard Structure */
struct hid_keyboard {
    struct usbhid_device *hid_dev;
    uint32_t report_length;
    
    /* Modifier keys descriptor */
    struct hid_data_descriptor modifier;
    
    /* Keys descriptor */
    struct hid_data_descriptor keys;
};

/* HID Keyboard Functions */
int hid_keyboard_open(struct usbhid_device *hid_dev, struct hid_keyboard *keyboard);
void hid_keyboard_close(struct hid_keyboard *keyboard);
int hid_keyboard_fetch_report(struct hid_keyboard *keyboard);

/* Key functions */
int hid_keyboard_get_key(struct hid_keyboard *keyboard, uint32_t key_code,
                        uint32_t *value, uint8_t is_last);
int hid_keyboard_set_key(struct hid_keyboard *keyboard, uint32_t key_code,
                        uint32_t value, uint8_t is_last);

/* Modifier functions */
int hid_keyboard_get_modifier(struct hid_keyboard *keyboard, uint32_t modifier,
                             uint32_t *value, uint8_t is_last);
int hid_keyboard_set_modifier(struct hid_keyboard *keyboard, uint32_t modifier,
                             uint32_t value, uint8_t is_last);

#endif /* HID_KEYBOARD_H */