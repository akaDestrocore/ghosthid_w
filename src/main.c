/* src/main.c
 *
 * Minimal Zephyr main — hid_bridge already spawns the worker threads (K_THREAD_DEFINE).
 * This file just logs startup and returns (leaves RTOS running).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

void main(void)
{
    LOG_INF("app: main started");
    /* hid_bridge.c already uses K_THREAD_DEFINE to spawn hid_bridge_start,
     * so there is nothing else to do here. */
}
