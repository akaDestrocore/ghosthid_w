#include <stdint.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(composite_hid_wrapper, LOG_LEVEL_DBG);

#include "composite_hid.h" /* path should match your include path */
#include "usbd_core.h"

extern USBDCompositeHID usbd_composite_hid;
extern USBD_HandleTypeDef hUsbDeviceFS;

int composite_hid_send_report(uint8_t interface_idx, uint8_t *report, uint16_t len)
{
    if (interface_idx >= usbd_composite_hid.interface_cnt) {
        LOG_ERR("invalid interface_idx %u", interface_idx);
        return -1;
    }
    if (!report || len == 0) {
        LOG_ERR("invalid report");
        return -1;
    }

    /* ensure USB device stack initialized */
    if (&hUsbDeviceFS == NULL || hUsbDeviceFS.pData == NULL) {
        LOG_ERR("USB device not initialized (call MX_USB_DEVICE_Init() first)");
        return -2;
    }

    uint8_t ep = usbd_composite_hid.ep_addr[interface_idx];
    LOG_DBG("TX report -> if=%u ep=0x%02X len=%u", interface_idx, ep, len);

    USBD_StatusTypeDef st = USBD_LL_Transmit(&hUsbDeviceFS, ep, report, len);
    if (st != USBD_OK) {
        LOG_ERR("USBD_LL_Transmit failed: %d", (int)st);
        return -3;
    }
    return 0;
}
