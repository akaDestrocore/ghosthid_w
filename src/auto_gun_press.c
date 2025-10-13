/* Auto Gun Press Implementation for Zephyr */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "auto_gun_press.h"

LOG_MODULE_REGISTER(agp, LOG_LEVEL_INF);

/* Raw data arrays - weapon recoil patterns */
static float AGP_RAW_DATA_NONE[] = {
    0, 0, 1000,
};

static float AGP_RAW_DATA_AK47[] = {
    0, 0, 30,
    -3.77885, -7.33026, 99,
    3.88382, -18.674, 99,
    -2.49497, -29.9552, 99,
    -1.05429, -31.9007, 99,
    12.3244, -33.1178, 99,
    8.45939, -27.7364, 99,
    13.1337, -19.8188, 99,
    -16.9038, -15.1133, 99,
    -42.0762, 5.02222, 99,
    -23.1968, -9.1423, 99,
    13.4584, -7.82848, 99,
    -16.283, -1.73109, 99,
    -26.8526, 9.52371, 99,
    -2.44215, -5.35626, 99,
    37.232, -8.02386, 99,
    23.2809, -5.40851, 99,
    14.3236, -5.75076, 99,
    26.6208, 2.82855, 99,
    34.9165, 8.60448, 99,
    -19.1055, -5.58987, 99,
    5.08387, 0.156464, 99,
    -8.60053, -5.91907, 99,
    -9.06119, -2.09341, 99,
    20.3148, 2.80682, 99,
    6.87328, -5.38808, 99,
    -21.3522, -0.609879, 99,
    -33.0195, 2.71869, 99,
    -47.1704, 21.1572, 99,
    -14.4156, -0.197525, 109,
};

static float AGP_RAW_DATA_M4A4[] = {
    0, 0, 20,
    0, 0, 88,
    1.3436535, -15.8159, 88,
    -4.98035, -16.096, 88,
    5.17241, -23.2379, 88,
    -7.64901, -26.6215, 88,
    -4.35077, -23.4396, 88,
    16.2926, -13.9699, 88,
    10.0171, -16.122, 88,
    24.4126, -5.73607, 88,
    -6.9532, -8.03185, 88,
    -17.891, -5.34038, 88,
    -27.7995, -0.496811, 88,
    -28.203, 2.01843, 88,
    -24.4919, 8.56436, 88,
    -0.139397, -3.24046, 88,
    7.04116, -9.62352, 88,
    -11.3982, 0.672928, 88,
    -12.562, 6.20432, 88,
    3.55202, -1.73409, 88,
    31.0881, -5.4948, 88,
    13.6174, -6.05292, 88,
    28.1378, 1.83047, 88,
    7.74411, -1.42644, 88,
    11.7364, -5.93803, 88,
    -11.0829, -3.38919, 88,
    5.83399, -0.696899, 98,
    4.18425, 1.05573, 88,
    2.55716, -4.91539, 88,
    5.49631, -3.339, 98,
};

struct agp_collect {
    float *data;
    int len;
    int multiple;
    int sleep_suber;
};

struct agp_context {
    struct agp_collect *collect;
    
    /* Parameters */
    float coefficient;
    float sensitive;
    
    /* Processed data arrays */
    int32_t *arr_x;
    int32_t *arr_y;
    int32_t *arr_ts;
    int arr_length;
    
    /* Runtime state */
    int arr_index;
    uint32_t last_tick_ms;
    
    struct k_mutex lock;
};

#define AGP_DATA_ARRAY_LEN 3

static struct agp_collect s_agp_collect_arr[AGP_DATA_ARRAY_LEN] = {
    {
        .data = AGP_RAW_DATA_NONE,
        .len = ARRAY_SIZE(AGP_RAW_DATA_NONE),
        .multiple = 1,
        .sleep_suber = 0
    },
    {
        .data = AGP_RAW_DATA_AK47,
        .len = ARRAY_SIZE(AGP_RAW_DATA_AK47),
        .multiple = 12,
        .sleep_suber = 0
    },
    {
        .data = AGP_RAW_DATA_M4A4,
        .len = ARRAY_SIZE(AGP_RAW_DATA_M4A4),
        .multiple = 12,
        .sleep_suber = 0
    },
};

static void free_fixed_data(struct agp_context *ctx)
{
    if (!ctx) {
        return;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    if (ctx->arr_x) {
        k_free(ctx->arr_x);
        ctx->arr_x = NULL;
    }
    if (ctx->arr_y) {
        k_free(ctx->arr_y);
        ctx->arr_y = NULL;
    }
    if (ctx->arr_ts) {
        k_free(ctx->arr_ts);
        ctx->arr_ts = NULL;
    }
    
    ctx->arr_length = 0;
    ctx->arr_index = 0;
    
    k_mutex_unlock(&ctx->lock);
}

static int generate_fixed_data(struct agp_context *ctx)
{
    struct agp_collect *coll;
    int raw_idx = 0;
    int idx = 0;
    int i, j;
    float x, y, ts;
    int32_t sx, sy, sts;
    int32_t fixx, fixy, fixts;
    float sumx = 0, sumy = 0, sumts = 0;
    float sumxo = 0, sumyo = 0, sumtso = 0;

    if (!ctx) {
        return -EINVAL;
    }
    coll = ctx->collect;
    if (!coll) {
        LOG_ERR("No collect selected");
        return -EINVAL;
    }

    if (coll->len < 3) {
        LOG_ERR("Collect data too small: len=%d", coll->len);
        return -EINVAL;
    }

    if (coll->multiple <= 0) {
        LOG_ERR("Invalid collect multiple: %d", coll->multiple);
        return -EINVAL;
    }

    if (ctx->sensitive == 0.0f) {
        LOG_ERR("Invalid sensitivity (zero) — aborting");
        return -EINVAL;
    }

    int samples = coll->len / 3;
    ctx->arr_length = coll->multiple * samples;  /* each sample expands into `multiple` items */

    if (ctx->arr_length <= 0) {
        LOG_ERR("Computed arr_length is zero: samples=%d, multiple=%d",
                samples, coll->multiple);
        return -EINVAL;
    }

    ctx->arr_x = k_malloc(sizeof(int32_t) * ctx->arr_length);
    ctx->arr_y = k_malloc(sizeof(int32_t) * ctx->arr_length);
    ctx->arr_ts = k_malloc(sizeof(int32_t) * ctx->arr_length);

    if (!ctx->arr_x || !ctx->arr_y || !ctx->arr_ts) {
        LOG_ERR("Failed to allocate fixed data arrays (len=%d)", ctx->arr_length);
        free_fixed_data(ctx);
        return -ENOMEM;
    }
    
    for (i = 0; i < coll->len / 3; i++) {
        x = coll->data[raw_idx + 0] * ctx->coefficient / ctx->sensitive;
        y = coll->data[raw_idx + 1] * ctx->coefficient / ctx->sensitive;
        ts = coll->data[raw_idx + 2];
        raw_idx += 3;
        
        sx = (int32_t)floorf(x / coll->multiple);
        sy = (int32_t)floorf(y / coll->multiple);
        sts = (int32_t)floorf(ts / coll->multiple);
        if (sts <= 0) {
            sts = 1;
        }
        
        sumx += sx * coll->multiple;
        sumy += sy * coll->multiple;
        sumts += sts * coll->multiple;
        
        sumxo += x;
        sumyo += y;
        sumtso += ts;
        
        fixx = (int32_t)roundf(sumxo - sumx);
        fixy = (int32_t)roundf(sumyo - sumy);
        fixts = (int32_t)roundf(sumtso - sumts);
        
        for (j = 0; j < coll->multiple; j++) {
            ctx->arr_x[idx] = sx;
            ctx->arr_y[idx] = sy;
            ctx->arr_ts[idx] = sts;
            
            if (fixx > 0) {
                ctx->arr_x[idx] += 1;
                sumx += 1;
                fixx--;
            }
            if (fixy > 0) {
                ctx->arr_y[idx] += 1;
                sumy += 1;
                fixy--;
            }
            if (fixts > 0) {
                ctx->arr_ts[idx] += 1;
                sumts += 1;
                fixts--;
            }
            
            idx++;
        }
    }
    
    return 0;
}

int agp_coefficient_change(struct agp_context *ctx, uint8_t is_add)
{
    if (!ctx) {
        return -EINVAL;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    ctx->coefficient += is_add ? 0.1f : -0.1f;
    LOG_INF("Coefficient: %.2f", ctx->coefficient);
    k_mutex_unlock(&ctx->lock);
    
    return 0;
}

int agp_sensitive_change(struct agp_context *ctx, uint8_t is_add)
{
    if (!ctx) {
        return -EINVAL;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    ctx->sensitive += is_add ? 0.1f : -0.1f;
    LOG_INF("Sensitivity: %.2f", ctx->sensitive);
    k_mutex_unlock(&ctx->lock);
    
    return 0;
}

int agp_restart(struct agp_context *ctx)
{
    if (!ctx) {
        return -EINVAL;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    ctx->last_tick_ms = k_uptime_get_32();
    ctx->arr_index = 0;
    k_mutex_unlock(&ctx->lock);
    
    return 0;
}

int agp_get_data(struct agp_context *ctx, struct agp_data *data)
{
    if (!ctx || !data) return -EINVAL;

    k_mutex_lock(&ctx->lock, K_FOREVER);

    if (!ctx->arr_ts || ctx->arr_length <= 0) {
        k_mutex_unlock(&ctx->lock);
        return -1;
    }

    if (ctx->arr_index >= ctx->arr_length) {
        k_mutex_unlock(&ctx->lock);
        return -1;
    }
    
    uint32_t now = k_uptime_get_32();
    if (now > ctx->last_tick_ms + ctx->arr_ts[ctx->arr_index]) {
        ctx->last_tick_ms += ctx->arr_ts[ctx->arr_index];
        
        data->x = ctx->arr_x[ctx->arr_index];
        data->y = -1 * ctx->arr_y[ctx->arr_index];
        
        ctx->arr_index++;
        k_mutex_unlock(&ctx->lock);
        return 0;
    }
    
    k_mutex_unlock(&ctx->lock);
    return -1;
}

int agp_set_collect(struct agp_context *ctx, uint32_t index)
{
    int ret;
    
    if (!ctx) {
        return -EINVAL;
    }
    
    if (index >= AGP_DATA_ARRAY_LEN) {
        LOG_ERR("Invalid collect index: %d", index);
        return -EINVAL;
    }
    
    k_mutex_lock(&ctx->lock, K_FOREVER);
    
    ctx->collect = &s_agp_collect_arr[index];
    free_fixed_data(ctx);
    ret = generate_fixed_data(ctx);
    
    k_mutex_unlock(&ctx->lock);
    
    if (ret < 0) {
        LOG_ERR("Failed to generate fixed data");
        return ret;
    }
    
    LOG_INF("Set collect index: %d", index);
    return 0;
}

void agp_close(struct agp_context *ctx)
{
    if (ctx) {
        free_fixed_data(ctx);
        k_free(ctx);
    }
}

int agp_open(struct agp_context **ctx)
{
    struct agp_context *new_ctx;
    
    if (!ctx) {
        return -EINVAL;
    }
    
    new_ctx = k_malloc(sizeof(struct agp_context));
    if (!new_ctx) {
        LOG_ERR("Failed to allocate AGP context");
        return -ENOMEM;
    }
    
    memset(new_ctx, 0, sizeof(struct agp_context));
    k_mutex_init(&new_ctx->lock);
    
    new_ctx->coefficient = DEFAULT_COEFFICIENT;
    new_ctx->sensitive = DEFAULT_SENSITIVE;
    
    int ret = agp_set_collect(new_ctx, AGP_COLLECT_IDX_AK47);
    if (ret < 0) {
        LOG_ERR("Failed to set collect at open: %d", ret);
        free_fixed_data(new_ctx);
        k_free(new_ctx);
        return ret;
    }
    agp_restart(new_ctx);
    
    *ctx = new_ctx;
    LOG_INF("AGP context initialized");
    
    return 0;
}