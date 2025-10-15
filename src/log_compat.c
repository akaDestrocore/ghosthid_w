#include "log_compat.h"
#include <stdio.h>
#include <stdarg.h>

#define LOCAL_PRINTF_BUF 256

/* Simple printf2 -> printk wrapper to keep ordering and avoid HAL_UART dependency */
void printf2(char *fmt, ...)
{
    char buf[LOCAL_PRINTF_BUF];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    /* single-line printk to preserve atomicity */
    printk("%s\n", buf);
}

/* Simple hex dump helper using printf2/printk */
void hex_dump(uint8_t *buf, int len)
{
    if (!buf || len <= 0) {
        return;
    }

    for (int i = 0; i < len; i += 16) {
        int pos = 0;
        char line[128];
        pos += snprintf(line + pos, sizeof(line) - pos, "%04x: ", i);
        for (int j = 0; j < 16 && (i + j) < len; ++j) {
            pos += snprintf(line + pos, sizeof(line) - pos, "%02x ", buf[i + j]);
        }
        printk("%s\n", line);
    }
}
