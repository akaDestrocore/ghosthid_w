#ifndef _ASYNC_REPORT_H
#define _ASYNC_REPORT_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

int composite_hid_send_report_async(uint8_t interface_num, const uint8_t *report, size_t len);

#endif /* _ASYNC_REPORT_H */