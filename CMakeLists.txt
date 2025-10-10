cmake_minimum_required(VERSION 3.28.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

target_sources(app PRIVATE
    src/main.c
    src/drivers/ch375.c
    src/drivers/ch375_host.c
    src/drivers/ch375_uart.c
    src/hid/hid_parser.c
    src/hid/hid_mouse.c
    src/hid/hid_keyboard.c
    src/usb/composite_hid.c
    src/auto_gun_press.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)