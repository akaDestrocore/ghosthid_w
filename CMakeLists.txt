cmake_minimum_required(VERSION 3.28.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

target_sources(app PRIVATE
    src/main.c
    src/auto_gun_press.c
    src/ch375_zephyr_adapter.c
    src/composite_hid_wrapper.c
    src/log_compat.c
    src/hid_bridge.c
    src/hal_shim.c
    Drivers/ch375/src/ch375.c
    Drivers/ch375/src/ch375_usbhost.c
    Drivers/ch375/src/hid/usbhid.c
    Drivers/ch375/src/hid/hid.c
    Drivers/ch375/src/hid/hid_mouse.c
    Drivers/ch375/src/hid/hid_keyboard.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c
    USB_DEVICE/App/composite_hid.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/Target/usbd_conf.c
)

# If you must add Cube HAL PCD/LL files (only add if linker complains):
# target_sources(app PRIVATE
#     Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
#     Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
#     Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
# )

# Include directories
target_include_directories(app PRIVATE
    include
    Drivers/ch375/include
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
)

zephyr_library_include_directories(
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/soc
)
