/* Composite HID Header for Zephyr */

#ifndef USB_COMPOSITE_HID_H
#define USB_COMPOSITE_HID_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* Maximum number of HID interfaces */
#define COMPOSITE_HID_MAX_INTERFACES 2

/* USB HID Class definitions */
#define USB_BCC_HID             0x03
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22

/* USB Descriptor types */
#define USB_DESC_DEVICE         0x01
#define USB_DESC_CONFIGURATION  0x02
#define USB_DESC_STRING         0x03
#define USB_DESC_INTERFACE      0x04
#define USB_DESC_ENDPOINT       0x05

/* USB Configuration attributes */
#define USB_SCD_SELF_POWERED    0xC0
#define USB_SCD_BUS_POWERED     0x80

/* USB Endpoint types */
#define USB_DC_EP_CONTROL       0x00
#define USB_DC_EP_ISOCHRONOUS   0x01
#define USB_DC_EP_BULK          0x02
#define USB_DC_EP_INTERRUPT     0x03

/* Function prototypes */
int composite_hid_register_interface(uint8_t interface_num,
                                    uint8_t *report_desc,
                                    uint16_t report_desc_len,
                                    uint8_t max_packet,
                                    uint8_t interval);

int composite_hid_init(void);

int hid_device_send_report(uint8_t interface_num, uint8_t *report, size_t len);

void composite_hid_cleanup(void);

#endif /* USB_COMPOSITE_HID_H */