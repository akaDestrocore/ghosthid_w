#ifndef HID_MOUSE_H
#define HID_MOUSE_H
#include "hid.h"
#include "usbhid.h"

#define HID_MOUSE_BUTTON_LEFT    0
#define HID_MOUSE_BUTTON_RIGHT   1
#define HID_MOUSE_BUTTON_MIDDLE  2
#define HID_MOUSE_BUTTON_EX_BASE 3

#define HID_MOUSE_AXIS_X 0
#define HID_MOUSE_AXIS_Y 1

typedef struct HIDMouse_t {
    USBHID_Device_t *hid_dev;

    HID_DataDescriptor_t button;
    HID_DataDescriptor_t orientation;
    uint32_t report_length;
} HIDMouse_t;

int hid_mouse_get_button(HIDMouse_t *dev, uint32_t button_num, uint32_t *value, uint8_t is_last);
int hid_mouse_set_button(HIDMouse_t *dev, uint32_t button_num, uint32_t value, uint8_t is_last);
int hid_mouse_get_orientation(HIDMouse_t *dev, uint32_t axis_num, int32_t *value, uint8_t is_last);
int hid_mouse_set_orientation(HIDMouse_t *dev, uint32_t axis_num, int32_t value, uint8_t is_last);

int hid_mouse_fetch_report(HIDMouse_t *dev);
void hid_mouse_close(HIDMouse_t *dev);
int hid_mouse_open(USBHID_Device_t *usbhid_dev, HIDMouse_t *dev);

#endif /* HID_MOUSE_H */