#ifndef COMPOSITE_HID_WRAPPER_H
#define COMPOSITE_HID_WRAPPER_H

#include <stdint.h>

/**
 * Send a raw HID report to the composite HID interface 'interface_idx'.
 * Returns 0 on success; negative on error.
 *
 * Note: Must be called from thread context (not IRQ). The underlying function
 * will call USBD_LL_Transmit() on the ST USB Device stack.
 */
int composite_hid_send_report(uint8_t interface_idx, const uint8_t *report, uint16_t len);

#endif /* COMPOSITE_HID_WRAPPER_H */
