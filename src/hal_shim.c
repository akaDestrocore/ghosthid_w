/* src/hal_shim.c
 *
 * Minimal shim to map small HAL helpers to Zephyr kernel.
 * This file intentionally provides simple implementations used by the Cube USB code.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

/* Minimal Error_Handler used by some Cube files */
void Error_Handler(void)
{
    /* Print and spin here so faults are visible */
    printk("Error_Handler() called\n");
    while (1) {
        k_msleep(1000);
    }
}
