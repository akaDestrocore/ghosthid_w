/* src/auto_gun_press.c - Zephyr port */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "auto_gun_press.h"
#include "auto_gun_press_data.h"

LOG_MODULE_REGISTER(agp, LOG_LEVEL_INF);

typedef struct AGPCollect {
    float *data;
    int len;
    int multiple;
    int sleepsuber;
} AGPCollect;

typedef struct AGPContext {
    AGPCollect *collect;
    float coefficient;
    float sensitive;
    int32_t *arr_x;
    int32_t *arr_y;
    int32_t *arr_ts;
    int arr_length;
    int arr_index;
    uint32_t last_tick_ms;
} AGPContext;

#define AGP_DATA_ARRAY_LEN 3

AGPCollect s_agp_collet_arr[AGP_DATA_ARRAY_LEN] = {
    {.data = AGP_RAW_DATA_NONE, .len = (sizeof(AGP_RAW_DATA_NONE) / sizeof(float)), .multiple = 1, .sleepsuber = 0},
    {.data = AGP_RAW_DATA_AK47, .len = (sizeof(AGP_RAW_DATA_AK47) / sizeof(float)), .multiple = 12, .sleepsuber = 0},
    {.data = AGP_RAW_DATA_M4A4, .len = (sizeof(AGP_RAW_DATA_M4A4) / sizeof(float)), .multiple = 12, .sleepsuber = 0},
};

int agp_coefficient_change(AGPContext *context, uint8_t is_add)
{
    if (context == NULL) {
        LOG_ERR("param context can't be NULL");
        return -1;
    }
    context->coefficient += (is_add ? 0.1f : -0.1f);
    LOG_INF("coefficient=%.2f", (double)context->coefficient);
    return 0;
}

int agp_sensitive_change(AGPContext *context, uint8_t is_add)
{
    if (context == NULL) {
        LOG_ERR("param context can't be NULL");
        return -1;
    }
    context->sensitive += (is_add ? 0.1f : -0.1f);
    LOG_INF("sensitive=%.2f", (double)context->sensitive);
    return 0;
}

int agp_restart(AGPContext *context)
{
    if (context == NULL) {
        LOG_ERR("param context can't be NULL");
        return -1;
    }
    context->last_tick_ms = k_uptime_get_32();
    context->arr_index = 0;
    return 0;
}

int agp_get_data(AGPContext *context, AGPData *data)
{
    if (context == NULL || data == NULL) {
        return -1;
    }

    if (context->arr_index >= context->arr_length) {
        return -1;
    }

    if (k_uptime_get_32() > context->last_tick_ms + context->arr_ts[context->arr_index]) {
        context->last_tick_ms += context->arr_ts[context->arr_index];

        data->x = context->arr_x[context->arr_index];
        data->y = -1 * context->arr_y[context->arr_index];
        
        context->arr_index++;
        return 0;
    }
    return -1;
}

static void free_fixed_data(AGPContext *ctx)
{
    if (!ctx) return;
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
}

static int generate_fixed_data(AGPContext *ctx)
{
    AGPCollect *coll = ctx->collect;
    int raw_idx = 0;
    int idx = 0;
    int i, j;
    float x, y, ts;
    int32_t sx, sy, sts;
    int32_t fixx, fixy, fixts;
    float sumx = 0, sumy = 0, sumts = 0;
    float sumxo = 0, sumyo = 0, sumtso = 0;

    ctx->arr_length = coll->multiple * coll->len / 3;
    ctx->arr_x = (int32_t *)k_malloc(sizeof(int32_t) * ctx->arr_length);
    ctx->arr_y = (int32_t *)k_malloc(sizeof(int32_t) * ctx->arr_length);
    ctx->arr_ts = (int32_t *)k_malloc(sizeof(int32_t) * ctx->arr_length);

    if (ctx->arr_x == NULL || ctx->arr_y == NULL || ctx->arr_ts == NULL) {
        LOG_ERR("allocate fixed data array failed, length=%d", ctx->arr_length);
        free_fixed_data(ctx);
        return -1;
    }

    for (i = 0; i < coll->len / 3; i++) {
        x = coll->data[raw_idx + 0] * ctx->coefficient / ctx->sensitive;
        y = coll->data[raw_idx + 1] * ctx->coefficient / ctx->sensitive;
        ts = coll->data[raw_idx + 2];
        raw_idx += 3;
        
        sx = (int32_t)floorf(x / coll->multiple);
        sy = (int32_t)floorf(y / coll->multiple);
        sts = (int32_t)floorf(ts / coll->multiple);

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

int agp_set_collect(AGPContext *context, uint32_t index)
{
    int ret;
    if (context == NULL) {
        LOG_ERR("param context can't be NULL");
        return -1;
    }
    if (index >= AGP_DATA_ARRAY_LEN) {
        LOG_ERR("param index(%u) is invalid", index);
        return -1;
    }
    context->collect = &s_agp_collet_arr[index];
    free_fixed_data(context);
    ret = generate_fixed_data(context);
    if (ret < 0) {
        LOG_ERR("generate fixed data failed");
        return -1;
    }
    return 0;
}

void agp_close(AGPContext *context)
{
    if (context) {
        free_fixed_data(context);
        k_free(context);
    }
}

int agp_open(AGPContext **context)
{
    AGPContext *ctx = NULL;
    if (context == NULL) {
        LOG_ERR("param context can't be NULL");
        return -1;
    }
    
    ctx = (AGPContext *)k_malloc(sizeof(AGPContext));
    if (ctx == NULL) {
        LOG_ERR("allocate agp context failed");
        return -1;
    }
    memset(ctx, 0, sizeof(AGPContext));

    ctx->coefficient = DEFAULT_COEFFICIENT;
    ctx->sensitive = DEFAULT_SENSITIVE;

    (void)agp_set_collect(ctx, AGP_COLLECT_IDX_AK47);
    (void)agp_restart(ctx);
    *context = ctx;
    return 0;
}