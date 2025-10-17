#ifndef USB_HID_DEVICE_H
#define USB_HID_DEVICE_H

#include <stdint.h>

int usb_hid_device_init(void);
/* interface_idx selects which exported device/interface to send to (0=mouse,1=keyboard). */
int usb_hid_send_report(uint8_t interface_idx, uint8_t *report, uint32_t report_len);

#endif /* USB_HID_DEVICE_H */
