/* CH375 UART Driver Header for Zephyr */

#ifndef CH375_UART_H
#define CH375_UART_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* CH375 Command/Data macros for 9-bit mode */
#define CH375_CMD(x)  ((uint16_t)((x) | 0x0100))
#define CH375_DATA(x) ((uint16_t)((x) | 0x0000))

/* CH375 Default Baudrates */
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_WORK_BAUDRATE     115200

/* Function prototypes for 9-bit UART support */
int uart_poll_out_u16(const struct device *dev, uint16_t out_data);
int uart_poll_in_u16(const struct device *dev, uint16_t *in_data);

/* Asynchronous UART data structure */
struct ch375_uart_data;

/* Asynchronous UART functions */
int ch375_uart_init_async(const struct device *dev, struct ch375_uart_data *data);
int ch375_uart_send_async(const struct device *dev, struct ch375_uart_data *data,
                          const uint16_t *buf, size_t len);
int ch375_uart_recv_async(const struct device *dev, struct ch375_uart_data *data,
                         uint16_t *buf, size_t *len, k_timeout_t timeout);

#endif /* CH375_UART_H */