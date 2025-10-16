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

    /* Ensure USBD handle looks initialized */
    if (hUsbDeviceFS.pData == NULL) {
        LOG_ERR("USB device not initialized (pData == NULL) - abort TX");
        return -2;
    }

    /* Ensure device is configured */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        LOG_WRN("USB device not configured yet (state=%d) - skip report", (int)hUsbDeviceFS.dev_state);
        return -2;
    }

    uint8_t ep = usbd_composite_hid.ep_addr[interface_idx];
    /* sanity-check endpoint address */
    if ((ep & 0x7F) >= 8) {
        LOG_ERR("endpoint index out of range: 0x%02X", ep);
        return -3;
    }

    LOG_DBG("composite_hid_send_report: TX report -> if=%u ep=0x%02X len=%u", interface_idx, ep, len);

    /* perform transmit */
    USBD_StatusTypeDef st = USBD_LL_Transmit(&hUsbDeviceFS, ep, report, len);
    if (st != USBD_OK) {
        LOG_ERR("USBD_LL_Transmit failed: %d", (int)st);
        return -4;
    }
    return 0;
}

