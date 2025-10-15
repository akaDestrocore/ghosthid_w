#define ENABLE_LOG
#include "log_compat.h"

#include <assert.h>
#include "usb.h"
#include "bswap.h"

#include "composite_hid.h"

USBDCompositeHID usbd_composite_hid = {0};


uint8_t *AppendEndpointDescriptor(uint8_t *start, uint8_t *end,
  uint8_t addr, uint16_t max_packet_size, uint8_t interval)
{
  assert(start);
  assert(end);
  USB_EndpointDescriptor_t *desc = (USB_EndpointDescriptor_t *)start;

  if (end - start < sizeof(USB_EndpointDescriptor_t)) {
    return NULL;
  }
  desc->bLength = sizeof(USB_EndpointDescriptor_t);
  desc->bDescriptorType = USB_DT_ENDPOINT;
  desc->bEndpointAddress = addr;
  desc->bmAttributes = USB_ENDPOINT_TRANSFER_TYPE_INTERRUPT;
  desc->wMaxPacketSize = cpu_to_le16(max_packet_size);
  desc->bInterval = interval;
  return start + sizeof(USB_EndpointDescriptor_t);
}

uint8_t *AppendHID_Descriptor_t(uint8_t *start, uint8_t *end, uint16_t report_desc_size)
{
  assert(start);
  assert(end);
  USB_HID_Descriptor_t_t *desc = (USB_HID_Descriptor_t_t *)start;

  if (end - start < sizeof(USB_HID_Descriptor_t_t)) {
    return NULL;
  }
  desc->bLength = sizeof(USB_HID_Descriptor_t_t);
  desc->bDescriptorType = USB_DT_HID;
  desc->bcdHID = cpu_to_le16(0x0111); // HID 1.11
  desc->bCountryCode = 0;
  desc->bNumDescriptors = 0x01;
  desc->bClassDescriptorType_e = USB_DT_REPORT;
  desc->wClassDescriptorLength = cpu_to_le16(report_desc_size);
  return start + sizeof(USB_HID_Descriptor_t_t);
}


uint8_t *AppendInterfaceDescriptor(uint8_t *start, uint8_t *end,
  uint8_t interface_num)
{
  assert(start);
  assert(end);
  USB_InterfaceDescriptor_t *desc = (USB_InterfaceDescriptor_t *)start;

  if (end - start < sizeof(USB_InterfaceDescriptor_t)) {
    return NULL;
  }
  desc->bLength = sizeof(USB_InterfaceDescriptor_t);
  desc->bDescriptorType = USB_DT_INTERFACE;
  desc->bInterfaceNumber = interface_num;
  desc->bAlternateSetting = 0;
  desc->bNumEndpoints = 1;
  desc->bInterfaceClass = USB_CLASS_HID;
  desc->bInterfaceSubClass = 0;
  desc->bInterfaceProtocol = 0;
  desc->iInterface = 0;
  return start + sizeof(USB_InterfaceDescriptor_t);
}

uint8_t *AppendConfigDescriptor(uint8_t *start, uint8_t *end,
  uint16_t total_length, uint8_t interface_cnt)
{
  assert(start);
  assert(end);
  USB_ConfigDescriptor_t *desc = (USB_ConfigDescriptor_t *)start;

  if (end  - start < sizeof(USB_ConfigDescriptor_t)) {
    return NULL;
  }
  desc->bLength = sizeof(USB_ConfigDescriptor_t);
  desc->bDescriptorType = USB_DT_CONFIG;
  desc->wTotalLength = cpu_to_le16(total_length);
  desc->bNumInterfaces = interface_cnt;
  desc->bConfigurationValue = 0x01;
  desc->iConfiguration = 0;
  desc->bmAttributes = USBD_SELF_POWERED == 1U ? 0xC0: 0x80;
  desc->MaxPower = USBD_MAX_POWER;
  return start + sizeof(USB_ConfigDescriptor_t);
}

uint16_t GetConfigDescTotalLength(uint8_t interface_count)
{
  uint16_t total_len = 0;
  // config desc
  total_len += sizeof(USB_ConfigDescriptor_t);
  // interface desc
  total_len += (interface_count * sizeof(USB_InterfaceDescriptor_t));
  // hid desc
  total_len += (interface_count * sizeof(USB_HID_Descriptor_t_t));
  // endpoint desc
  total_len += (interface_count * sizeof(USB_EndpointDescriptor_t));
  return total_len;
}

uint8_t GetInterfaceCnt()
{
  uint8_t cnt = 0;
  int i;
  for (i = 0; i < COMPOSITE_HID_INTERFACE_NUM; i++) {
    if (usbd_composite_hid.report_desc[i]) {
      cnt++;
    }
  }
  return cnt;
}

uint8_t USBD_COMPOSITE_HID_Init()
{
  uint8_t interface_cnt = GetInterfaceCnt();
  uint16_t config_desc_len = GetConfigDescTotalLength(interface_cnt);
  uint8_t *config_desc_buf = NULL;
  uint8_t *config_desc_buf_end;
  uint8_t *cursor;
  int i;

  if (interface_cnt == 0) {
    ERROR("interface count can't be zero, please register inetface first");
    return (uint8_t)USBD_FAIL;
  }

  // allocate config descriptor buffer
  config_desc_buf = (uint8_t *)malloc(config_desc_len);
  if (config_desc_buf == NULL) {
    ERROR("allocate config descriptor buffer(len=%d) faield", config_desc_len);
    return (uint8_t)USBD_FAIL;
  }
  config_desc_buf_end = config_desc_buf + config_desc_len;
  
  cursor = AppendConfigDescriptor(config_desc_buf, config_desc_buf_end,
    config_desc_len, interface_cnt);
  for (i = 0; i < interface_cnt; i++) {
    uint16_t report_desc_len = usbd_composite_hid.report_desc_len[i];
    uint8_t ep_addr = usbd_composite_hid.ep_addr[i];
    uint8_t max_pack = usbd_composite_hid.max_pack[i];
    uint8_t interval = usbd_composite_hid.interval[i];
    cursor = AppendInterfaceDescriptor(cursor, config_desc_buf_end, (uint8_t)i);

    usbd_composite_hid.hid_desc[i] = cursor;

    cursor = AppendHID_Descriptor_t(cursor, config_desc_buf_end, report_desc_len);
    cursor = AppendEndpointDescriptor(cursor, config_desc_buf_end,
      ep_addr, max_pack, interval);
  }
  assert(cursor);
  usbd_composite_hid.interface_cnt = interface_cnt;
  usbd_composite_hid.config_desc = config_desc_buf;
  usbd_composite_hid.config_desc_len = config_desc_len;
  return (uint8_t)USBD_OK;
}


uint8_t USBD_COMPOSITE_HID_InterfaceRegister(uint8_t interface_num,
  uint8_t *hid_desc, uint8_t *report_descport, uint16_t length,
  uint8_t max_pack, uint8_t interval)
{
  if (interface_num >= COMPOSITE_HID_INTERFACE_NUM) {
    return (uint8_t)USBD_FAIL;
  }
  if (report_descport == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  usbd_composite_hid.hid_desc[interface_num] = hid_desc;
  usbd_composite_hid.report_desc[interface_num] = report_descport;
  usbd_composite_hid.report_desc_len[interface_num] = length;
  usbd_composite_hid.ep_addr[interface_num] = (uint8_t)((interface_num + 1) | USB_ENDPOINT_IN);
  usbd_composite_hid.max_pack[interface_num] = max_pack;
  usbd_composite_hid.interval[interface_num] = interval;
  return (uint8_t)USBD_OK;
}