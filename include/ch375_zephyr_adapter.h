#ifndef CH375_ZEPHYR_ADAPTER_H
#define CH375_ZEPHYR_ADAPTER_H

#include <stdint.h>
#include "ch375_interface.h"

/**
 * @brief Open CH375 on a Zephyr UART + INT GPIO
 *
 * @param uart_label   Zephyr device name for UART (e.g. "UART_2", "UART_3")
 * @param gpio_label   Zephyr device name for GPIO controller (e.g. "GPIOA")
 * @param int_pin      GPIO pin number for CH375 INT line
 * @param ctx_out      output pointer to CH375_Context_t* (allocated inside ch375_openContext)
 *
 * @return CH375_SUCCESS on success, other CH375_* error codes on failure
 */
int ch375_zephyr_open(const char *uart_label,
                      const char *gpio_label,
                      uint32_t int_pin,
                      CH375_Context_t **ctx_out);

/**
 * @brief Close CH375 context opened via ch375_zephyr_open
 *
 * @param ctx CH375_Context_t* (will be freed)
 */
int ch375_zephyr_close(CH375_Context_t *ctx);

#endif /* CH375_ZEPHYR_ADAPTER_H */
