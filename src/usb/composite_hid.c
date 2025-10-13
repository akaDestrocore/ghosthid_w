/* Composite HID Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "usb/composite_hid.h"

LOG_MODULE_REGISTER(composite_hid, LOG_LEVEL_INF);

#define COMPOSITE_HID_MAX_INTERFACES 2
#define HID_EP_SIZE 8
#define HID_REPORT_MAX_SIZE 256
static struct k_work send_report_work;
static uint8_t send_report_buf[HID_REPORT_MAX_SIZE];
static size_t  send_report_len;
static uint8_t send_report_iface;
bool composite_hid_usb_configured = false;

static atomic_t g_usb_configured = ATOMIC_INIT(0);

struct composite_hid_interface {
    uint8_t *report_desc;
    uint16_t report_desc_len;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t max_packet;
    uint8_t interval;
    bool configured;
};

static struct composite_hid_interface interfaces[COMPOSITE_HID_MAX_INTERFACES];
static uint8_t interface_count;
static uint8_t config_desc[256];
static uint16_t config_desc_len;

/* HID class specific descriptors */
struct hid_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bClassDescriptorType;
    uint16_t wClassDescriptorLength;
} __packed;

void composite_hid_set_usb_configured(bool v)
{
    /* update both atomic and the visible boolean */
    atomic_set(&g_usb_configured, v ? 1 : 0);
    composite_hid_usb_configured = v;
}


bool composite_hid_is_configured(void)
{
    return (bool)atomic_get(&g_usb_configured);
}

static void dump_buf(const uint8_t *buf, size_t len)
{
    char s[64];
    int p = 0;
    for (size_t i = 0; i < len && p + 3 < sizeof(s); i++) {
        p += snprintk(s + p, sizeof(s) - p, "%02x ", buf[i]);
    }
    LOG_DBG("report: %s", s);
}

static void send_report_work_handler(struct k_work *work)
{
    int ret;
    /* This runs in thread context — safe to call USB stack functions */
    ret = hid_device_send_report(send_report_iface, send_report_buf, send_report_len);
    if (ret) {
        LOG_ERR("hid_device_send_report failed: %d", ret);
    }
}

static inline bool usb_is_configured(void)
{
    return (bool)atomic_get(&g_usb_configured);
}

/* Build configuration descriptor */
static int build_config_descriptor(void)
{
    uint8_t *ptr = config_desc;
    uint16_t total_len = 0;
    
    /* Configuration descriptor */
    struct usb_cfg_descriptor *cfg = (struct usb_cfg_descriptor *)ptr;
    cfg->bLength = sizeof(struct usb_cfg_descriptor);
    cfg->bDescriptorType = USB_DT_CONFIGURATION;
    cfg->wTotalLength = 0;  /* Will be updated later */
    cfg->bNumInterfaces = interface_count;
    cfg->bConfigurationValue = 1;
    cfg->iConfiguration = 0;
    cfg->bmAttributes = USB_SCD_SELF_POWERED;
    cfg->bMaxPower = 50;  /* 100mA */
    
    ptr += sizeof(struct usb_cfg_descriptor);
    total_len += sizeof(struct usb_cfg_descriptor);
    
    /* Add interface descriptors */
    for (int i = 0; i < interface_count; i++) {
        struct composite_hid_interface *iface = &interfaces[i];
        
        /* Interface descriptor */
        struct usb_if_descriptor *if_desc = (struct usb_if_descriptor *)ptr;
        if_desc->bLength = sizeof(struct usb_if_descriptor);
        if_desc->bDescriptorType = USB_DESC_INTERFACE;
        if_desc->bInterfaceNumber = i;
        if_desc->bAlternateSetting = 0;
        if_desc->bNumEndpoints = 1;
        if_desc->bInterfaceClass = USB_BCC_HID;
        if_desc->bInterfaceSubClass = 0;
        if_desc->bInterfaceProtocol = 0;
        if_desc->iInterface = 0;
        
        ptr += sizeof(struct usb_if_descriptor);
        total_len += sizeof(struct usb_if_descriptor);
        
        /* HID descriptor */
        struct hid_descriptor *hid_desc = (struct hid_descriptor *)ptr;
        hid_desc->bLength = sizeof(struct hid_descriptor);
        hid_desc->bDescriptorType = USB_DESC_HID;
        hid_desc->bcdHID = sys_cpu_to_le16(0x0111);  /* HID 1.11 */
        hid_desc->bCountryCode = 0;
        hid_desc->bNumDescriptors = 1;
        hid_desc->bClassDescriptorType = USB_DESC_HID_REPORT;
        hid_desc->wClassDescriptorLength = sys_cpu_to_le16(iface->report_desc_len);
        
        ptr += sizeof(struct hid_descriptor);
        total_len += sizeof(struct hid_descriptor);
        
        /* Endpoint descriptor */
        struct usb_ep_descriptor *ep_desc = (struct usb_ep_descriptor *)ptr;
        ep_desc->bLength = sizeof(struct usb_ep_descriptor);
        ep_desc->bDescriptorType = USB_DESC_ENDPOINT;
        ep_desc->bEndpointAddress = iface->ep_in;
        ep_desc->bmAttributes = USB_DC_EP_INTERRUPT;
        ep_desc->wMaxPacketSize = sys_cpu_to_le16(iface->max_packet);
        ep_desc->bInterval = iface->interval;
        
        ptr += sizeof(struct usb_ep_descriptor);
        total_len += sizeof(struct usb_ep_descriptor);
    }
    
    /* Update total length */
    cfg->wTotalLength = sys_cpu_to_le16(total_len);
    config_desc_len = total_len;
    
    return 0;
}

int composite_hid_register_interface(uint8_t interface_num,
                                    uint8_t *report_desc,
                                    uint16_t report_desc_len,
                                    uint8_t max_packet,
                                    uint8_t interval)
{
    if (interface_num >= COMPOSITE_HID_MAX_INTERFACES) {
        LOG_ERR("Interface number %d exceeds maximum", interface_num);
        return -EINVAL;
    }
    
    if (!report_desc || report_desc_len == 0) {
        LOG_ERR("Invalid report descriptor");
        return -EINVAL;
    }
    
    struct composite_hid_interface *iface = &interfaces[interface_num];
    
    iface->report_desc = k_malloc(report_desc_len);
    if (!iface->report_desc) {
        LOG_ERR("Failed to allocate report descriptor");
        return -ENOMEM;
    }
    
    memcpy(iface->report_desc, report_desc, report_desc_len);
    iface->report_desc_len = report_desc_len;
    iface->ep_in = 0x80 | (interface_num + 1);
    iface->ep_out = interface_num + 1;
    iface->max_packet = max_packet;
    iface->interval = interval;
    iface->configured = true;
    
    if (interface_num >= interface_count) {
        interface_count = interface_num + 1;
    }
    
    LOG_INF("Registered HID interface %d (EP IN: 0x%02x, len: %d)",
           interface_num, iface->ep_in, report_desc_len);
    
    return 0;
}

int composite_hid_init(void)
{
    int ret;
    
    k_work_init(&send_report_work, send_report_work_handler);

    if (interface_count == 0) {
        LOG_ERR("No interfaces registered");
        return -EINVAL;
    }
    
    /* Build configuration descriptor */
    ret = build_config_descriptor();
    if (ret) {
        LOG_ERR("Failed to build configuration descriptor");
        return ret;
    }
    
    LOG_INF("Composite HID initialized with %d interfaces", interface_count);
    return 0;
}

int hid_device_send_report(uint8_t interface_num, uint8_t *report, size_t len)
{
    struct composite_hid_interface *iface;
    int ret;

    if (interface_num >= interface_count) {
        return -EINVAL;
    }

    iface = &interfaces[interface_num];
    if (!iface->configured) { /* means "registered" */
        return -ENODEV;
    }

    if (!usb_is_configured()) {
        LOG_WRN("USB not configured; skipping HID send on iface %d", interface_num);
        return -EAGAIN;
    }

    if (iface->max_packet == 0) {
        LOG_ERR("Interface %d has max_packet == 0; aborting send", interface_num);
        return -EINVAL;
    }

    if (len > iface->max_packet) {
        LOG_WRN("Truncating report len %zu to max_packet %u", len, iface->max_packet);
        len = iface->max_packet;
    }

    LOG_DBG("hid_device_send_report iface=%d len=%zu usb_config=%d max_packet=%u",
        interface_num, len, usb_is_configured(), iface->max_packet);

    dump_buf(report, len);
    ret = usb_write(iface->ep_in, report, len, NULL);
    if (ret) {
        LOG_ERR("Failed to send HID report on interface %d: %d", interface_num, ret);
        return ret;
    }

    LOG_DBG("usb_write -> %d (sent %zu bytes)", ret, len);

    return 0;
}

void composite_hid_cleanup(void)
{
    for (int i = 0; i < COMPOSITE_HID_MAX_INTERFACES; i++) {
        if (interfaces[i].report_desc) {
            k_free(interfaces[i].report_desc);
            interfaces[i].report_desc = NULL;
        }
        interfaces[i].configured = false;
    }
    interface_count = 0;
}