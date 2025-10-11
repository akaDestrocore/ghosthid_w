/* CH375 UART Header for Zephyr - Native 9-bit Support */

#ifndef CH375_UART_H
#define CH375_UART_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* 
 * CH375 Command/Data macros for 9-bit mode
 * Bit 8 = 1 for commands, Bit 8 = 0 for data
 */
#define CH375_CMD(x)  ((uint16_t)((x) | 0x0100))
#define CH375_DATA(x) ((uint16_t)((x) | 0x0000))

/* CH375 Default Baudrates */
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_WORK_BAUDRATE     115200

/*
 * Configure UART for 9-bit mode
 * 
 * This configures the UART hardware to use 9 data bits (UART_CFG_DATA_BITS_9)
 * which is required for CH375 communication.
 * 
 * @param dev UART device
 * @param baudrate Desired baudrate (9600 or 115200 for CH375)
 * @return 0 on success, negative errno on failure
 */
int ch375_uart_configure_9bit(const struct device *dev, uint32_t baudrate);

/*
 * Write 16-bit data with timeout
 * 
 * Helper function that wraps uart_poll_out_u16() with timeout support.
 * 
 * @param dev UART device
 * @param data 16-bit data (9 bits used)
 * @param timeout Timeout duration
 * @return 0 on success, negative errno on failure
 */
int ch375_uart_write_u16_timeout(const struct device *dev, uint16_t data, k_timeout_t timeout);

/*
 * Read 16-bit data with timeout
 * 
 * Helper function that wraps uart_poll_in_u16() with timeout support.
 * 
 * @param dev UART device
 * @param data Pointer to store received 16-bit data
 * @param timeout Timeout duration
 * @return 0 on success, negative errno on failure
 */
int ch375_uart_read_u16_timeout(const struct device *dev, uint16_t *data, k_timeout_t timeout);

/*
 * NOTE: For direct 9-bit UART access without timeout, use Zephyr's native functions:
 * 
 * void uart_poll_out_u16(const struct device *dev, uint16_t out_u16);
 * int uart_poll_in_u16(const struct device *dev, uint16_t *p_u16);
 * 
 * These are available when CONFIG_UART_WIDE_DATA=y is enabled in prj.conf
 */

#endif /* CH375_UART_H */