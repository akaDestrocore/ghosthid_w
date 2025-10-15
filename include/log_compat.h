#ifndef LOG_COMPAT_H
#define LOG_COMPAT_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdarg.h>

/* Lightweight project-wide logging macros mapped to printk.
 * These are intentionally NOT using Zephyr LOG_MODULE_REGISTER/DECLARE to avoid
 * the multiple-definition problem when included in headers.
 *
 * If you later want full Zephyr logging features, you can refactor:
 * - put LOG_MODULE_REGISTER(app_log, ...) into exactly one .c
 * - add LOG_MODULE_DECLARE(app_log) at the top of each .c that uses LOG_*
 */

#define INFO(fmt, ...)  printk("[INFO] " fmt "\n", ##__VA_ARGS__)
#define ERROR(fmt, ...) printk("[ERROR] " fmt "\n", ##__VA_ARGS__)
#define WARN(fmt, ...)  printk("[WARN] " fmt "\n", ##__VA_ARGS__)
#ifdef ENABLE_DEBUG
#define DEBUG(fmt, ...) printk("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG(fmt, ...) do { } while (0)
#endif

/* Backwards compatible functions used in legacy code */
void printf2(char *fmt, ...);
void hex_dump(uint8_t *buf, int len);

#endif /* LOG_COMPAT_H */
