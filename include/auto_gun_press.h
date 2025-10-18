/* Auto Gun Press Header for Zephyr */

#ifndef AUTO_GUN_PRESS_H
#define AUTO_GUN_PRESS_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* AGP Collection Indices */
#define AGP_COLLECT_IDX_NONE 0
#define AGP_COLLECT_IDX_AK47 1
#define AGP_COLLECT_IDX_M4A4 2

/* Default Parameters */
#define DEFAULT_COEFFICIENT 1.0f
#define DEFAULT_SENSITIVE   2.5f

/* AGP Data Structure */
struct agp_data {
    int32_t x;
    int32_t y;
};

/* Forward declaration */
struct agp_context;

/* AGP Functions */
int agp_open(struct agp_context **ctx);
void agp_close(struct agp_context *ctx);
int agp_restart(struct agp_context *ctx);
int agp_get_data(struct agp_context *ctx, struct agp_data *data);
int agp_set_collect(struct agp_context *ctx, uint32_t index);
int agp_coefficient_change(struct agp_context *ctx, uint8_t is_add);
int agp_sensitive_change(struct agp_context *ctx, uint8_t is_add);

#endif /* AUTO_GUN_PRESS_H */