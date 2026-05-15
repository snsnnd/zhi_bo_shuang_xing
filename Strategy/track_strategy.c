#include "track_strategy.h"
#include "../Common/car_config.h"

static uint32_t g_lost_since_ms;
static bool g_lost_active;
static float g_last_valid_err;

void track_strategy_init(void) {
    g_lost_since_ms = 0U;
    g_lost_active = false;
    g_last_valid_err = 0.0f;
}

static bool line_lost(line_bin_t b) { return !b.l2 && !b.l1 && !b.r1 && !b.r2; }

strategy_output_t track_strategy_step(float line_error, line_bin_t bin, imu_state_t imu, encoder_state_t enc, uint32_t tick_ms, const map_event_t *current_event) {
    (void)enc;
    strategy_output_t out = { .mode = CAR_MODE_TRACKING, .target_speed_mps = SPEED_MID_MPS, .steer_limit = 350.0f, .lost_timeout = false };

    if (!line_lost(bin)) {
        g_last_valid_err = line_error;
        g_lost_active = false;
    }

    if (line_lost(bin)) {
        if (!g_lost_active) {
            g_lost_active = true;
            g_lost_since_ms = tick_ms;
        }
        out.mode = CAR_MODE_LOST;
        out.target_speed_mps = SPEED_LOW_MPS;
        out.steer_limit = 450.0f;
        if ((tick_ms - g_lost_since_ms) > LINE_LOST_TIMEOUT_MS) {
            out.mode = CAR_MODE_PROTECT;
            out.target_speed_mps = 0.0f;
            out.steer_limit = 0.0f;
            out.lost_timeout = true;
        }
        return out;
    }

    if ((bin.l2 && !bin.l1) || (bin.r2 && !bin.r1)) {
        out.mode = CAR_MODE_HAIRPIN;
        out.target_speed_mps = SPEED_LOW_MPS;
        out.steer_limit = 500.0f;
        return out;
    }

    if (line_error < STRAIGHT_ERROR_TH && line_error > -STRAIGHT_ERROR_TH &&
        imu.yaw_rate_filtered < STRAIGHT_YAW_RATE_TH_DPS && imu.yaw_rate_filtered > -STRAIGHT_YAW_RATE_TH_DPS) {
        out.mode = CAR_MODE_PREDICTION;
        out.target_speed_mps = current_event ? current_event->recommend_speed_mps : SPEED_HIGH_MPS;
        out.steer_limit = 300.0f;
    }
    return out;
}
