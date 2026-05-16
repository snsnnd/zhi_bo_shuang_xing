#include "track_strategy.h"
#include "../Common/car_config.h"

#if CAR_ENABLE_LOST_PROTECTION
static uint32_t g_lost_since_ms;
#endif
static bool g_lost_active;
static float g_last_valid_err;

// Reset strategy memory before a new run or after mode reinitialization.
void track_strategy_init(void) {
#if CAR_ENABLE_LOST_PROTECTION
    g_lost_since_ms = 0U;
#endif
    g_lost_active = false;
    g_last_valid_err = 0.0f;
}

// 丢线判定：四路全白
static bool line_lost(line_bin_t b) { return !b.l2 && !b.l1 && !b.r1 && !b.r2; }

// 策略决策：处理丢线、急弯、预测等模式并输出目标速度/转向限幅
strategy_output_t track_strategy_step(float line_error, line_bin_t bin, imu_state_t imu, encoder_state_t enc, uint32_t tick_ms, const map_event_t *current_event) {
    (void)enc;
    strategy_output_t out = { .mode = CAR_MODE_TRACKING, .target_speed_mps = SPEED_MID_MPS, .steer_limit = 350.0f, .search_error = line_error, .lost_timeout = false };

    if (!line_lost(bin)) {
        g_last_valid_err = line_error;
        g_lost_active = false;
    }

    if (line_lost(bin)) {
        if (!g_lost_active) {
            g_lost_active = true;
#if CAR_ENABLE_LOST_PROTECTION
            g_lost_since_ms = tick_ms;
#endif
        }
        out.mode = CAR_MODE_LOST;
        out.target_speed_mps = SPEED_LOW_MPS;
        out.steer_limit = 450.0f;
        // 丢线时当前误差会变成无效大值，搜索方向使用最后一次有效误差。
        out.search_error = g_last_valid_err;
        // 超过丢线超时阈值，进入保护停车；初期调车可关闭，避免传感器未调好时频繁保护。
#if CAR_ENABLE_LOST_PROTECTION
        if ((tick_ms - g_lost_since_ms) > LINE_LOST_TIMEOUT_MS) {
            out.mode = CAR_MODE_PROTECT;
            out.target_speed_mps = 0.0f;
            out.steer_limit = 0.0f;
            out.lost_timeout = true;
        }
#else
        (void)tick_ms;
#endif
        return out;
    }

    if ((bin.l2 && !bin.l1) || (bin.r2 && !bin.r1)) {
        out.mode = CAR_MODE_HAIRPIN;
        out.target_speed_mps = SPEED_LOW_MPS;
        out.steer_limit = 500.0f;
        return out;
    }

#if CAR_ENABLE_MAP_PREDICTION
    if (line_error < STRAIGHT_ERROR_TH && line_error > -STRAIGHT_ERROR_TH &&
        imu.yaw_rate_filtered < STRAIGHT_YAW_RATE_TH_DPS && imu.yaw_rate_filtered > -STRAIGHT_YAW_RATE_TH_DPS) {
        out.mode = CAR_MODE_PREDICTION;
        out.target_speed_mps = current_event ? current_event->recommend_speed_mps : SPEED_HIGH_MPS;
        out.steer_limit = 300.0f;
    }
#else
    (void)imu;
    (void)current_event;
#endif
    return out;
}
