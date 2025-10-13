cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

# Add source files
target_sources(app PRIVATE
    src/main.c
    src/drivers/ch375.c
    src/drivers/ch375_host.c
    src/drivers/ch375_uart.c
    src/hid/hid_parser.c
    src/hid/hid_mouse.c
    src/hid/hid_keyboard.c
    src/hid/async_report.c
    src/usb/composite_hid.c
    src/auto_gun_press.c
)

# Add include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against STM32 LL libraries for direct UART access
# These are needed for 9-bit UART configuration
zephyr_library_include_directories(
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/soc
)