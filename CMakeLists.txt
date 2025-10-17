cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ghosthid_w)

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ch375.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ch375_uart_parity.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ch375_zephyr_glue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_hid_forwarder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ch375_usbhost.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/usbhid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/hid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/hid_mouse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/hid_keyboard.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
