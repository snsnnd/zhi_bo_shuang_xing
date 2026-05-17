#include "app_car.h"

#include "main.h"
#include "../Common/car_config.h"
#include "../Common/runtime_config.h"
#include "../Common/car_types.h"
#include "../BSP/bsp_encoder.h"
#include "../BSP/bsp_imu.h"
#include "../BSP/bsp_line.h"
#include "../BSP/bsp_motor.h"
#include "../BSP/bsp_oled.h"
#if CAR_ENABLE_HC05
#include "../BSP/bsp_hc05.h"
#endif
#include "../Control/line_pid.h"
#if CAR_ENABLE_REMOTE_CONTROL
#include "../Control/remote_control.h"
#endif
#include "../Control/speed_pid.h"
#if CAR_ENABLE_PID_SCOPE
#include "../Debug/PIDScope/pid_scope_adapter.h"
#endif
#include "../Strategy/track_map.h"
#include "../Strategy/track_strategy.h"

// ------------------------- 全局控制对象 -------------------------
// 循迹 PID：将 line_error 映射到转向量 steer
static line_pid_t g_line_pid;
// 左/右轮速度 PID：将目标速度映射到 PWM
static speed_pid_t g_lspd_pid;
static speed_pid_t g_rspd_pid;

// 当前整车模式（学习/循迹/预测/丢线/保护等）
static car_mode_t g_mode = CAR_MODE_IDLE;
// 系统毫秒节拍（10ms 周期累加）
static uint32_t g_tick_ms;
static volatile uint8_t g_control_due;
static volatile uint8_t g_control_tick_div;
static volatile uint32_t g_control_overrun;
static oled_runtime_t g_rt;
static bool g_rt_valid;
// 第一圈地图学习上下文
static map_learning_ctx_t g_learn_ctx;
// 是否已完成第一圈地图学习
static bool g_learning_done;

// 限幅工具函数：防止控制量越界
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float speed_to_open_loop_pwm(float target_mps) {
    return clampf((target_mps / SPEED_HIGH_MPS) * CAR_MAX_PWM, CAR_MIN_PWM, CAR_MAX_PWM);
}

static void speed_pid_observe_open_loop(speed_pid_t *pid, float target, float feedback, float output) {
    if (!pid) return;
    pid->last_target = target;
    pid->last_feedback = feedback;
    pid->last_error = target - feedback;
    pid->last_output = output;
    pid->last_p_term = pid->kp * pid->last_error;
    pid->last_i_term = pid->integral;
    pid->last_d_term = 0.0f;
}

static float speed_pid_update_with_feedforward(speed_pid_t *pid, float target, float feedback) {
    float pid_out = speed_pid_update(pid, target, feedback);
    float ff = (target > 0.0f) ? (0.55f * speed_to_open_loop_pwm(target)) : 0.0f;
    float out = clampf(pid_out + ff, CAR_MIN_PWM, CAR_MAX_PWM);
    pid->last_output = out;
    return out;
}

#if CAR_ENABLE_PID_SCOPE
enum {
    PID_STATUS_MOTOR_ENABLE = 1U << 0,
    PID_STATUS_MOTOR_OUTPUT = 1U << 1,
    PID_STATUS_SPEED_PID = 1U << 2,
    PID_STATUS_TUNE = 1U << 3,
    PID_STATUS_OUTPUT_SAT = 1U << 4,
    PID_STATUS_PROTECT = 1U << 5,
    PID_STATUS_LOST = 1U << 6,
    PID_STATUS_MAP_PREDICTION = 1U << 7
};
#endif

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
static void apply_remote_pid_update(const remote_pid_update_t *u) {
    if (!u || !u->pending) return;

    if (u->target == REMOTE_PID_LINE) {
        g_line_pid.kp = u->kp;
        g_line_pid.ki = u->ki;
        g_line_pid.kd = u->kd;
        line_pid_reset(&g_line_pid);
    }

    if (u->target == REMOTE_PID_SPEED_LEFT || u->target == REMOTE_PID_SPEED_BOTH) {
        g_lspd_pid.kp = u->kp;
        g_lspd_pid.ki = u->ki;
        g_lspd_pid.kd = u->kd;
        speed_pid_reset(&g_lspd_pid);
    }
    if (u->target == REMOTE_PID_SPEED_RIGHT || u->target == REMOTE_PID_SPEED_BOTH) {
        g_rspd_pid.kp = u->kp;
        g_rspd_pid.ki = u->ki;
        g_rspd_pid.kd = u->kd;
        speed_pid_reset(&g_rspd_pid);
    }

    remote_control_clear_pid_update();
}
#endif

#if CAR_ENABLE_HC05 && (CAR_ENABLE_PID_SCOPE_OVER_HC05 || CAR_ENABLE_REMOTE_CONTROL)
static void hc05_app_rx(uint8_t byte) {
#if CAR_ENABLE_PID_SCOPE && CAR_ENABLE_PID_SCOPE_OVER_HC05
    if (runtime_feature_enabled(RC_FEATURE_PID_SCOPE) && runtime_feature_enabled(RC_FEATURE_PID_SCOPE_OVER_HC05)) pid_scope_rx_byte(byte);
#endif
#if CAR_ENABLE_REMOTE_CONTROL
    remote_control_rx_byte(byte);
#endif
}

static int hc05_write_text(const char *text) { return bsp_hc05_send_text(text); }
#endif

void app_car_on_1ms_tick(void) {
    g_control_tick_div++;
    if (g_control_tick_div >= 10U) {
        g_control_tick_div = 0U;
        if (g_control_due != 0U) {
            g_control_overrun++;
        } else {
            g_control_due = 1U;
        }
    }
}

uint32_t app_car_get_control_overrun(void) { return g_control_overrun; }

void app_car_run_scheduler(void) {
    if (g_control_due != 0U) {
        __disable_irq();
        g_control_due = 0U;
        __enable_irq();
        app_car_run_10ms();
    } else {
        app_car_run_background();
    }
}

// 整车初始化：初始化 PID、IMU 和策略模块。地图只保存在 RAM，并通过上位机保存。
void app_car_init(void) {
    runtime_config_init();
    // 1) 初始化循迹/速度 PID 参数（这里是默认参数，后续建议通过标定页在线调）
    line_pid_init(&g_line_pid, 120.0f, 0.0f, 25.0f, CAR_CTRL_DT_S, -350.0f, 350.0f);
    speed_pid_init(&g_lspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);
    speed_pid_init(&g_rspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);

#if CAR_ENABLE_HC05
#if CAR_ENABLE_PID_SCOPE_OVER_HC05 || CAR_ENABLE_REMOTE_CONTROL
    hc05_port_t hc05_port = { bsp_hc05_port_write, bsp_hc05_port_time_ms, hc05_app_rx };
    bsp_hc05_init(&hc05_port);
#else
    bsp_hc05_init(0);
#endif
#endif

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    remote_control_port_t rc_port = { hc05_write_text, bsp_hc05_port_time_ms };
    remote_control_init(&rc_port);
#endif

#if CAR_ENABLE_PID_SCOPE
    pid_scope_init();
    pid_scope_bind_line_pid(&g_line_pid);
    pid_scope_bind_left_speed_pid(&g_lspd_pid);
    pid_scope_bind_right_speed_pid(&g_rspd_pid);
#endif

    // 2) 初始化 IMU 与策略模块；IMU 可通过开关关闭，便于先测循迹/电机。
#if CAR_ENABLE_IMU
    bsp_imu_init(CAR_CTRL_DT_S);
#endif
    track_strategy_init();

    // 3) 地图学习是高级功能，默认关闭；学习结果通过上位机通信保存，不再写 MCU Flash。
    track_map_reset();
    g_learning_done = false;
    if (runtime_feature_enabled(RC_FEATURE_TRACK_MAP_LEARNING)) {
        track_map_learning_init(&g_learn_ctx, 0.0f, 0.0f);
        g_mode = CAR_MODE_LEARNING;
    } else {
        g_mode = CAR_MODE_TRACKING;
    }

    // 4) 复位系统节拍
    g_tick_ms = 0U;
    g_control_due = 0U;
    g_control_tick_div = 0U;
    g_control_overrun = 0U;
    g_rt_valid = false;
}

// 10ms 主控制循环（建议由定时中断或高优先级任务稳定驱动）
// 执行顺序：采样 -> 学习/定位 -> 策略决策 -> PID 控制 -> 电机输出 -> OLED 显示
void app_car_run_10ms(void) {
    // 周期节拍推进
    g_tick_ms += 10U;

    // ---------- A. 采样层：读取已滤波后的传感器状态 ----------
    // 注意：bsp_line_get_raw() 返回的是经过 LPF 的值，不是裸 ADC。
    line_raw_t line_raw;
    line_bin_t line_bin;
    float line_err;
    if (runtime_feature_enabled(RC_FEATURE_LINE_SENSOR)) {
        line_raw = bsp_line_get_raw();
    // 二值判线状态（黑/白）用于状态机与急弯/丢线判定
        line_bin = bsp_line_get_bin();
    // 循迹误差（已轻度滤波）用于 line PID
        line_err = bsp_line_get_error();
    } else {
    // 关闭循迹时假定车辆在线中央，便于单独测试 IMU/OLED/电机链路。
        line_raw = (line_raw_t){ .l2 = 0.0f, .l1 = 1.0f, .r1 = 1.0f, .r2 = 0.0f };
        line_bin = (line_bin_t){ .l2 = false, .l1 = true, .r1 = true, .r2 = false };
        line_err = 0.0f;
    }
    // IMU 当前姿态/角速度状态
    imu_state_t imu = runtime_feature_enabled(RC_FEATURE_IMU) ? bsp_imu_get_state() : (imu_state_t){0};
    // 编码器速度与里程（速度已滤波，里程直通）
    encoder_state_t enc = runtime_feature_enabled(RC_FEATURE_ENCODER) ? bsp_encoder_get_state() : (encoder_state_t){0};

    // ---------- B. 学习层：第一圈自动识别地图事件 ----------
    // 条件：仅在未学习完成时运行
    if (runtime_feature_enabled(RC_FEATURE_TRACK_MAP_LEARNING) && !g_learning_done) {
        // 持续喂给学习状态机：根据误差趋势 / yaw 变化 / 外侧触发自动识别路段
        (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, false);

        // 这里用 5m 作为示例“首圈完成”条件；实车建议替换为起终点标志或圈计数
        if (enc.distance_m > 5.0f) {
            // 强制收尾，确保最后一个路段也能加入 RAM 事件表，后续由上位机保存。
            (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, true);
            g_learning_done = true;
        }
    }

    // ---------- C. 定位层：基于里程查询当前与下一事件 ----------
    const map_event_t *ev = runtime_feature_enabled(RC_FEATURE_MAP_PREDICTION) ? track_map_find_current(enc.distance_m) : 0;
    const map_event_t *next_ev = runtime_feature_enabled(RC_FEATURE_MAP_PREDICTION) ? track_map_find_next(enc.distance_m) : 0;

    // ---------- D. 策略层：计算目标模式/速度/转向限幅 ----------
    strategy_output_t so = track_strategy_step(line_err, line_bin, imu, enc, g_tick_ms, ev);
    g_mode = so.mode;

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    remote_control_state_t rc = remote_control_get_state();
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL)) {
        apply_remote_pid_update(&rc.pid_update);
        if (rc.encoder_query_pending) remote_control_reply_encoder(enc.left_speed_mps, enc.right_speed_mps, enc.distance_m);
#if CAR_ENABLE_PID_SCOPE
        pid_scope_set_selected_channel(rc.scope_device_id, rc.scope_channel_id);
#endif
        if (rc.mode_override) g_mode = rc.mode;
        if (rc.speed_override) so.target_speed_mps = rc.target_speed_mps;
        if (rc.speed_limit_override) so.target_speed_mps = clampf(so.target_speed_mps, 0.0f, rc.speed_limit_mps);
        if (rc.force_stop) g_mode = CAR_MODE_PROTECT;
    }
#endif

    // ---------- E. 控制层：循迹 PID + 双速度 PID ----------
    // 1) 由 line error 得到转向量 steer
    float steer = line_pid_update(&g_line_pid, line_err);
    // 按当前模式限幅（例如急弯允许更大转向，预测时适当限制）
    steer = clampf(steer, -so.steer_limit, so.steer_limit);

    // 2) 将转向量分配到左右轮目标速度（差速转向）
    float left_target = so.target_speed_mps - 0.0015f * steer;
    float right_target = so.target_speed_mps + 0.0015f * steer;

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL) && rc.speed_tune_mode != REMOTE_SPEED_TUNE_NONE) {
        left_target = 0.0f;
        right_target = 0.0f;
        if (rc.speed_tune_mode == REMOTE_SPEED_TUNE_LEFT || rc.speed_tune_mode == REMOTE_SPEED_TUNE_BOTH) left_target = rc.target_speed_mps;
        if (rc.speed_tune_mode == REMOTE_SPEED_TUNE_RIGHT || rc.speed_tune_mode == REMOTE_SPEED_TUNE_BOTH) right_target = rc.target_speed_mps;
    }
#endif

    // 3) 模式特化控制
    if (g_mode == CAR_MODE_LOST) {
        // 丢线模式：按最后一次有效误差方向搜索，避免 99.0f 无效误差固定偏向一侧。
        float search = (so.search_error > 0.0f ? 1.0f : -1.0f);
        left_target = SPEED_LOW_MPS - 0.08f * search;
        right_target = SPEED_LOW_MPS + 0.08f * search;
    } else if (g_mode == CAR_MODE_PROTECT) {
        // 保护模式：停车 + 清理循迹 PID 积分/历史，避免恢复时冲击
        left_target = 0.0f;
        right_target = 0.0f;
        line_pid_reset(&g_line_pid);
    }

    // 4) 左右轮速度闭环 -> PWM
    float left_pwm;
    float right_pwm;
    if (runtime_feature_enabled(RC_FEATURE_SPEED_PID)) {
        left_pwm = speed_pid_update_with_feedforward(&g_lspd_pid, left_target, enc.left_speed_mps);
        right_pwm = speed_pid_update_with_feedforward(&g_rspd_pid, right_target, enc.right_speed_mps);
    } else {
    // 速度闭环未验证前，使用速度目标到 PWM 的简单前馈，降低调试复杂度。
        left_pwm = speed_to_open_loop_pwm(left_target);
        right_pwm = speed_to_open_loop_pwm(right_target);
        speed_pid_observe_open_loop(&g_lspd_pid, left_target, enc.left_speed_mps, left_pwm);
        speed_pid_observe_open_loop(&g_rspd_pid, right_target, enc.right_speed_mps, right_pwm);
    }

    // ---------- F. 执行层：输出电机命令 ----------
    motor_cmd_t cmd = {
        .left_pwm = clampf(left_pwm, CAR_MIN_PWM, CAR_MAX_PWM),
        .right_pwm = clampf(right_pwm, CAR_MIN_PWM, CAR_MAX_PWM),
        .left_dir = true,
        .right_dir = true
    };

    // 保护模式强制为 0，防止任何残余命令输出
    if (g_mode == CAR_MODE_PROTECT) {
        cmd.left_pwm = 0.0f;
        cmd.right_pwm = 0.0f;
    }

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL) && rc.motor_test_mode != REMOTE_MOTOR_TEST_NONE) {
        cmd.left_pwm = 0.0f;
        cmd.right_pwm = 0.0f;
        cmd.left_dir = true;
        cmd.right_dir = true;
        if (rc.motor_test_mode == REMOTE_MOTOR_TEST_LEFT || rc.motor_test_mode == REMOTE_MOTOR_TEST_BOTH) cmd.left_pwm = rc.motor_test_pwm;
        if (rc.motor_test_mode == REMOTE_MOTOR_TEST_RIGHT || rc.motor_test_mode == REMOTE_MOTOR_TEST_BOTH) cmd.right_pwm = rc.motor_test_pwm;
        speed_pid_observe_open_loop(&g_lspd_pid, 0.0f, enc.left_speed_mps, cmd.left_pwm);
        speed_pid_observe_open_loop(&g_rspd_pid, 0.0f, enc.right_speed_mps, cmd.right_pwm);
    }
#endif

    if (runtime_feature_enabled(RC_FEATURE_MOTOR_OUTPUT)) {
#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
        if (!runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL) || (rc.motor_enable && !rc.force_stop)) {
            bsp_motor_set(cmd);
        } else {
            motor_cmd_t safe_cmd = { .left_pwm = 0.0f, .right_pwm = 0.0f, .left_dir = true, .right_dir = true };
            bsp_motor_set(safe_cmd);
            cmd = safe_cmd;
        }
#else
        bsp_motor_set(cmd);
#endif
    } else {
    // 电机输出关闭时仍下发 0 PWM，保证切换阶段不会残留上一次命令。
    motor_cmd_t safe_cmd = { .left_pwm = 0.0f, .right_pwm = 0.0f, .left_dir = true, .right_dir = true };
    bsp_motor_set(safe_cmd);
    cmd = safe_cmd;
    }

#if CAR_ENABLE_PID_SCOPE
    uint16_t pid_status = 0U;
    if (runtime_feature_enabled(RC_FEATURE_MOTOR_OUTPUT)) pid_status |= PID_STATUS_MOTOR_OUTPUT;
    if (runtime_feature_enabled(RC_FEATURE_SPEED_PID)) pid_status |= PID_STATUS_SPEED_PID;
    if (runtime_feature_enabled(RC_FEATURE_MAP_PREDICTION)) pid_status |= PID_STATUS_MAP_PREDICTION;
    if (g_mode == CAR_MODE_PROTECT) pid_status |= PID_STATUS_PROTECT;
    if (g_mode == CAR_MODE_LOST) pid_status |= PID_STATUS_LOST;
    if (cmd.left_pwm >= (CAR_MAX_PWM - 1.0f) || cmd.right_pwm >= (CAR_MAX_PWM - 1.0f)) pid_status |= PID_STATUS_OUTPUT_SAT;
#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL) && rc.motor_enable) pid_status |= PID_STATUS_MOTOR_ENABLE;
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL) && rc.speed_tune_mode != REMOTE_SPEED_TUNE_NONE) pid_status |= PID_STATUS_TUNE;
#endif
    pid_scope_set_vehicle_state(enc.distance_m, imu.yaw_deg, pid_status);
    pid_scope_set_map_state(ev ? ev->id : 0xFFFFU,
                            ev ? (uint8_t)ev->type : (uint8_t)SEG_UNKNOWN,
                            next_ev ? next_ev->id : 0xFFFFU,
                            next_ev ? (uint8_t)next_ev->type : (uint8_t)SEG_UNKNOWN,
                            next_ev ? (next_ev->start_dist_m - enc.distance_m) : -1.0f,
                            (uint8_t)g_mode);
#endif

    // ---------- G. 缓存后台调试数据 ----------
    // OLED/PIDScope 属于慢任务，只缓存最新数据，后台空闲时再发送/刷新。
    g_rt.page = 0U;
    g_rt.line_raw = line_raw;
    g_rt.line_bin = line_bin;
    g_rt.line_err = line_err;
    g_rt.left_pwm = cmd.left_pwm;
    g_rt.right_pwm = cmd.right_pwm;
    g_rt.left_speed = enc.left_speed_mps;
    g_rt.right_speed = enc.right_speed_mps;
    g_rt.distance_m = enc.distance_m;
    g_rt.yaw_deg = imu.yaw_deg;
    g_rt.segment_id = ev ? ev->id : 0xFFFFU;
    g_rt.segment_type = ev ? ev->type : SEG_UNKNOWN;
    g_rt.dist_to_next_event = next_ev ? (next_ev->start_dist_m - enc.distance_m) : -1.0f;
    g_rt.current_limit_speed = so.target_speed_mps;
    g_rt_valid = true;
}

void app_car_run_background(void) {
#if CAR_ENABLE_HC05
    if (runtime_feature_enabled(RC_FEATURE_HC05)) bsp_hc05_poll();
#endif

#if CAR_ENABLE_REMOTE_CONTROL && CAR_ENABLE_HC05
    if (runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL)) remote_control_poll();
#endif

#if CAR_ENABLE_OLED
    if (runtime_feature_enabled(RC_FEATURE_OLED)) {
#if CAR_ENABLE_HC05 && CAR_ENABLE_HC05_OLED_TEST
        if (runtime_feature_enabled(RC_FEATURE_HC05_OLED_TEST)) bsp_oled_show_hc05_activity(bsp_hc05_rx_count(), bsp_hc05_tx_count());
        else if (g_rt_valid) bsp_oled_show_runtime(g_mode, &g_rt);
#else
        if (g_rt_valid) bsp_oled_show_runtime(g_mode, &g_rt);
#endif
    }
#endif

#if CAR_ENABLE_PID_SCOPE
    if (runtime_feature_enabled(RC_FEATURE_PID_SCOPE)) pid_scope_poll_10ms();
#endif
}
