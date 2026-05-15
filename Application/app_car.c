#include "app_car.h"

#include "../BSP/bsp_encoder.h"
#include "../BSP/bsp_imu.h"
#include "../BSP/bsp_line.h"
#include "../BSP/bsp_motor.h"
#include "../BSP/bsp_oled.h"
#include "../Common/car_config.h"
#include "../Common/car_types.h"
#include "../Control/line_pid.h"
#include "../Control/speed_pid.h"
#include "../Strategy/track_map.h"
#include "../Strategy/track_strategy.h"

static line_pid_t g_line_pid;
static speed_pid_t g_lspd_pid;
static speed_pid_t g_rspd_pid;
static car_mode_t g_mode = CAR_MODE_IDLE;
static uint32_t g_tick_ms;
static map_learning_ctx_t g_learn_ctx;
static bool g_learning_done;

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void app_car_init(void) {
    line_pid_init(&g_line_pid, 120.0f, 0.0f, 25.0f, CAR_CTRL_DT_S, -350.0f, 350.0f);
    speed_pid_init(&g_lspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);
    speed_pid_init(&g_rspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);
    bsp_imu_init(CAR_CTRL_DT_S);
    track_strategy_init();
    track_map_reset();
    g_learning_done = false;
    if (track_map_load_from_flash()) {
        g_learning_done = true;
        g_mode = CAR_MODE_TRACKING;
    } else {
        track_map_learning_init(&g_learn_ctx, 0.0f, 0.0f);
        g_mode = CAR_MODE_LEARNING;
    }
    g_tick_ms = 0U;
}

void app_car_run_10ms(void) {
    g_tick_ms += 10U;
    line_raw_t line_raw = bsp_line_get_raw();
    line_bin_t line_bin = bsp_line_get_bin();
    float line_err = bsp_line_get_error();
    imu_state_t imu = bsp_imu_get_state();
    encoder_state_t enc = bsp_encoder_get_state();

    if (!g_learning_done) {
        (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, false);
        if (enc.distance_m > 5.0f) {
            (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, true);
            (void)track_map_save_to_flash();
            g_learning_done = true;
        }
    }

    const map_event_t *ev = track_map_find_current(enc.distance_m);
    const map_event_t *next_ev = track_map_find_next(enc.distance_m);
    strategy_output_t so = track_strategy_step(line_err, line_bin, imu, enc, g_tick_ms, ev);
    g_mode = so.mode;

    float steer = line_pid_update(&g_line_pid, line_err);
    steer = clampf(steer, -so.steer_limit, so.steer_limit);
    float left_target = so.target_speed_mps - 0.0015f * steer;
    float right_target = so.target_speed_mps + 0.0015f * steer;

    if (g_mode == CAR_MODE_LOST) {
        float search = (line_err > 0.0f ? 1.0f : -1.0f);
        left_target = SPEED_LOW_MPS - 0.08f * search;
        right_target = SPEED_LOW_MPS + 0.08f * search;
    } else if (g_mode == CAR_MODE_PROTECT) {
        left_target = 0.0f;
        right_target = 0.0f;
        line_pid_reset(&g_line_pid);
    }

    float left_pwm = speed_pid_update(&g_lspd_pid, left_target, enc.left_speed_mps);
    float right_pwm = speed_pid_update(&g_rspd_pid, right_target, enc.right_speed_mps);

    motor_cmd_t cmd = {
        .left_pwm = clampf(left_pwm, CAR_MIN_PWM, CAR_MAX_PWM),
        .right_pwm = clampf(right_pwm, CAR_MIN_PWM, CAR_MAX_PWM),
        .left_dir = true,
        .right_dir = true
    };

    if (g_mode == CAR_MODE_PROTECT) {
        cmd.left_pwm = 0.0f;
        cmd.right_pwm = 0.0f;
    }

    bsp_motor_set(cmd);

    oled_runtime_t rt;
    rt.page = 0U;
    rt.line_raw = line_raw;
    rt.line_bin = line_bin;
    rt.line_err = line_err;
    rt.left_pwm = cmd.left_pwm;
    rt.right_pwm = cmd.right_pwm;
    rt.left_speed = enc.left_speed_mps;
    rt.right_speed = enc.right_speed_mps;
    rt.distance_m = enc.distance_m;
    rt.yaw_deg = imu.yaw_deg;
    rt.segment_id = ev ? ev->id : 0xFFFFU;
    rt.segment_type = ev ? ev->type : SEG_UNKNOWN;
    rt.dist_to_next_event = next_ev ? (next_ev->start_dist_m - enc.distance_m) : -1.0f;
    rt.current_limit_speed = so.target_speed_mps;
    bsp_oled_show_runtime(g_mode, &rt);
}
