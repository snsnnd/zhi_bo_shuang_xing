#include "track_map.h"
#include "../Common/car_config.h"

static map_event_t g_events[MAP_MAX_EVENTS];
static uint16_t g_count;

void track_map_reset(void) { g_count = 0U; }

bool track_map_add_event(map_event_t event) {
    if (g_count >= MAP_MAX_EVENTS) return false;
    event.id = g_count;
    g_events[g_count++] = event;
    return true;
}

const map_event_t *track_map_find_current(float distance_m) {
    for (uint16_t i = 0; i < g_count; ++i) {
        if (distance_m >= g_events[i].start_dist_m && distance_m <= g_events[i].end_dist_m) return &g_events[i];
    }
    return 0;
}

const map_event_t *track_map_find_next(float distance_m) {
    for (uint16_t i = 0; i < g_count; ++i) {
        if (distance_m < g_events[i].start_dist_m) return &g_events[i];
    }
    return 0;
}

uint16_t track_map_count(void) { return g_count; }

static float fabsf_local(float x) { return x < 0.0f ? -x : x; }

static segment_type_t detect_type(float yaw_delta, float err, uint8_t flips, line_bin_t bin) {
    if ((bin.l2 && !bin.l1) || (bin.r2 && !bin.r1)) return bin.l2 ? SEG_SHARP_LEFT : SEG_SHARP_RIGHT;
    if (fabsf_local(yaw_delta) > 130.0f) return SEG_U_TURN;
    if (flips >= 2U) return SEG_S_CURVE;
    if (err > 0.8f) return SEG_CURVE_RIGHT;
    if (err < -0.8f) return SEG_CURVE_LEFT;
    return SEG_STRAIGHT;
}

static float speed_by_type(segment_type_t t) {
    if (t == SEG_STRAIGHT) return SPEED_HIGH_MPS;
    if (t == SEG_S_CURVE || t == SEG_U_TURN) return SPEED_MID_LOW_MPS;
    if (t == SEG_SHARP_LEFT || t == SEG_SHARP_RIGHT) return SPEED_LOW_MPS;
    return SPEED_MID_MPS;
}

void track_map_learning_init(map_learning_ctx_t *ctx, float start_dist, float yaw_deg) {
    ctx->current_type = SEG_UNKNOWN;
    ctx->start_dist_m = start_dist;
    ctx->prev_err = 0.0f;
    ctx->yaw_start_deg = yaw_deg;
    ctx->err_sign_flips = 0U;
}

bool track_map_learning_step(map_learning_ctx_t *ctx, float distance_m, float yaw_deg, float line_err, line_bin_t bin, bool force_finalize) {
    float yaw_delta = yaw_deg - ctx->yaw_start_deg;
    if ((line_err > 0.0f && ctx->prev_err < 0.0f) || (line_err < 0.0f && ctx->prev_err > 0.0f)) ctx->err_sign_flips++;
    segment_type_t new_type = detect_type(yaw_delta, line_err, ctx->err_sign_flips, bin);

    bool changed = (ctx->current_type != SEG_UNKNOWN && new_type != ctx->current_type);
    bool long_enough = (distance_m - ctx->start_dist_m) > 0.08f;
    if ((changed && long_enough) || force_finalize) {
        map_event_t e;
        e.start_dist_m = ctx->start_dist_m;
        e.end_dist_m = distance_m;
        e.type = ctx->current_type == SEG_UNKNOWN ? new_type : ctx->current_type;
        e.recommend_speed_mps = speed_by_type(e.type);
        e.feedforward = (e.type == SEG_CURVE_LEFT || e.type == SEG_SHARP_LEFT) ? -0.2f : (e.type == SEG_CURVE_RIGHT || e.type == SEG_SHARP_RIGHT) ? 0.2f : 0.0f;
        e.risk = (e.type == SEG_U_TURN || e.type == SEG_SHARP_LEFT || e.type == SEG_SHARP_RIGHT) ? 2U : (e.type == SEG_S_CURVE ? 1U : 0U);
        (void)track_map_add_event(e);

        ctx->start_dist_m = distance_m;
        ctx->yaw_start_deg = yaw_deg;
        ctx->err_sign_flips = 0U;
        ctx->current_type = new_type;
        ctx->prev_err = line_err;
        return true;
    }

    ctx->current_type = new_type;
    ctx->prev_err = line_err;
    return false;
}
