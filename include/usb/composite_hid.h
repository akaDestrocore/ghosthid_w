/* src/usb/composite_hid.h
 *
 * Minimal composite HID device API used by your main code.
 *
 * main.c expects:
 *  - composite_hid_register_interface(interface_num, report_desc, report_desc_len, max_packet, interval)
 *  - composite_hid_init()
 *  - hid_device_send_report(interface_num, report_buf, len)
 *  - composite_hid_cleanup()
 *
 * We provide those symbols (note: main.c calls hid_device_send_report; we implement it too).
 */

#ifndef _COMPOSITE_HID_H_
#define _COMPOSITE_HID_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int composite_hid_register_interface(uint8_t interface_num,
                                     uint8_t *report_desc,
                                     uint16_t report_desc_len,
                                     uint8_t max_packet,
                                     uint8_t interval);

int composite_hid_init(void);

/* Send a HID report for the given registered interface number.
 * Returns 0 on success, negative errno on error.
 */
int hid_device_send_report(uint8_t interface_num, uint8_t *report, size_t len);

void composite_hid_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* _COMPOSITE_HID_H_ */
