#include "app_car.h"

#include "main.h"
#include "../BSP/bsp_encoder.h"
#include "../BSP/bsp_imu.h"
#include "../BSP/bsp_line.h"
#include "../BSP/bsp_motor.h"
#include "../BSP/bsp_oled.h"
#include "../Common/car_config.h"
#include "../Common/car_types.h"
#include "../Control/line_pid.h"
#include "../Control/speed_pid.h"
#if CAR_ENABLE_PID_SCOPE
#include "../Debug/PIDScope/pid_scope_adapter.h"
#endif
#include "../Strategy/track_map.h"
#include "../Strategy/track_strategy.h"

// ------------------------- 全局控制对象 -------------------------
// 循迹 PID：将 line_error 映射到转向量 steer
static line_pid_t g_line_pid;
#if CAR_ENABLE_SPEED_PID
// 左/右轮速度 PID：将目标速度映射到 PWM
static speed_pid_t g_lspd_pid;
static speed_pid_t g_rspd_pid;
#endif

// 当前整车模式（学习/循迹/预测/丢线/保护等）
static car_mode_t g_mode = CAR_MODE_IDLE;
// 系统毫秒节拍（10ms 周期累加）
static uint32_t g_tick_ms;
static volatile uint8_t g_control_due;
static volatile uint8_t g_control_tick_div;
static volatile uint32_t g_control_overrun;
static oled_runtime_t g_rt;
static bool g_rt_valid;
#if CAR_ENABLE_TRACK_MAP_LEARNING
// 第一圈地图学习上下文
static map_learning_ctx_t g_learn_ctx;
// 是否已完成学习（或已从 Flash 成功加载）
static bool g_learning_done;
#endif

// 限幅工具函数：防止控制量越界
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

#if !CAR_ENABLE_SPEED_PID
static float speed_to_open_loop_pwm(float target_mps) {
    return clampf((target_mps / SPEED_HIGH_MPS) * CAR_MAX_PWM, CAR_MIN_PWM, CAR_MAX_PWM);
}
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

// 整车初始化：初始化 PID、IMU、策略模块，并尝试从 Flash 读取地图
void app_car_init(void) {
    // 1) 初始化循迹/速度 PID 参数（这里是默认参数，后续建议通过标定页在线调）
    line_pid_init(&g_line_pid, 120.0f, 0.0f, 25.0f, CAR_CTRL_DT_S, -350.0f, 350.0f);
#if CAR_ENABLE_SPEED_PID
    speed_pid_init(&g_lspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);
    speed_pid_init(&g_rspd_pid, 500.0f, 50.0f, 0.0f, CAR_CTRL_DT_S, 0.0f, CAR_MAX_PWM);
#endif

#if CAR_ENABLE_PID_SCOPE
    pid_scope_init();
    pid_scope_bind_line_pid(&g_line_pid);
#if CAR_ENABLE_SPEED_PID
    pid_scope_bind_left_speed_pid(&g_lspd_pid);
    pid_scope_bind_right_speed_pid(&g_rspd_pid);
#endif
#endif

    // 2) 初始化 IMU 与策略模块；IMU 可通过开关关闭，便于先测循迹/电机。
#if CAR_ENABLE_IMU
    bsp_imu_init(CAR_CTRL_DT_S);
#endif
    track_strategy_init();

    // 3) 地图学习/Flash 都是高级功能，默认关闭，避免初期调车时写 Flash 或进入学习状态。
    track_map_reset();
#if CAR_ENABLE_TRACK_MAP_LEARNING
    g_learning_done = false;
#if CAR_ENABLE_TRACK_MAP_FLASH
    if (track_map_load_from_flash()) {
        // 加载成功：直接进入循迹/预测主流程，可跳过第一圈学习
        g_learning_done = true;
        g_mode = CAR_MODE_TRACKING;
    } else {
#endif
        // 加载失败：进入学习模式，等待首圈自动打点
        track_map_learning_init(&g_learn_ctx, 0.0f, 0.0f);
        g_mode = CAR_MODE_LEARNING;
#if CAR_ENABLE_TRACK_MAP_FLASH
    }
#endif
#else
    g_mode = CAR_MODE_TRACKING;
#endif

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
#if CAR_ENABLE_LINE_SENSOR
    line_raw_t line_raw = bsp_line_get_raw();
    // 二值判线状态（黑/白）用于状态机与急弯/丢线判定
    line_bin_t line_bin = bsp_line_get_bin();
    // 循迹误差（已轻度滤波）用于 line PID
    float line_err = bsp_line_get_error();
#else
    // 关闭循迹时假定车辆在线中央，便于单独测试 IMU/OLED/电机链路。
    line_raw_t line_raw = { .l2 = 0.0f, .l1 = 1.0f, .r1 = 1.0f, .r2 = 0.0f };
    line_bin_t line_bin = { .l2 = false, .l1 = true, .r1 = true, .r2 = false };
    float line_err = 0.0f;
#endif
    // IMU 当前姿态/角速度状态
#if CAR_ENABLE_IMU
    imu_state_t imu = bsp_imu_get_state();
#else
    imu_state_t imu = {0};
#endif
    // 编码器速度与里程（速度已滤波，里程直通）
#if CAR_ENABLE_ENCODER
    encoder_state_t enc = bsp_encoder_get_state();
#else
    encoder_state_t enc = {0};
#endif

    // ---------- B. 学习层：第一圈自动识别地图事件 ----------
    // 条件：仅在未学习完成时运行
#if CAR_ENABLE_TRACK_MAP_LEARNING
    if (!g_learning_done) {
        // 持续喂给学习状态机：根据误差趋势 / yaw 变化 / 外侧触发自动识别路段
        (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, false);

        // 这里用 5m 作为示例“首圈完成”条件；实车建议替换为起终点标志或圈计数
        if (enc.distance_m > 5.0f) {
            // 强制收尾，确保最后一个路段也能落盘
            (void)track_map_learning_step(&g_learn_ctx, enc.distance_m, imu.yaw_deg, line_err, line_bin, true);
            // 保存到 Flash，供下次上电直接加载
#if CAR_ENABLE_TRACK_MAP_FLASH
            (void)track_map_save_to_flash();
#endif
            g_learning_done = true;
        }
    }
#endif

    // ---------- C. 定位层：基于里程查询当前与下一事件 ----------
#if CAR_ENABLE_MAP_PREDICTION
    const map_event_t *ev = track_map_find_current(enc.distance_m);
    const map_event_t *next_ev = track_map_find_next(enc.distance_m);
#else
    const map_event_t *ev = 0;
    const map_event_t *next_ev = 0;
#endif

    // ---------- D. 策略层：计算目标模式/速度/转向限幅 ----------
    strategy_output_t so = track_strategy_step(line_err, line_bin, imu, enc, g_tick_ms, ev);
    g_mode = so.mode;

    // ---------- E. 控制层：循迹 PID + 双速度 PID ----------
    // 1) 由 line error 得到转向量 steer
    float steer = line_pid_update(&g_line_pid, line_err);
    // 按当前模式限幅（例如急弯允许更大转向，预测时适当限制）
    steer = clampf(steer, -so.steer_limit, so.steer_limit);

    // 2) 将转向量分配到左右轮目标速度（差速转向）
    float left_target = so.target_speed_mps - 0.0015f * steer;
    float right_target = so.target_speed_mps + 0.0015f * steer;

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
#if CAR_ENABLE_SPEED_PID
    float left_pwm = speed_pid_update(&g_lspd_pid, left_target, enc.left_speed_mps);
    float right_pwm = speed_pid_update(&g_rspd_pid, right_target, enc.right_speed_mps);
#else
    // 速度闭环未验证前，使用速度目标到 PWM 的简单前馈，降低调试复杂度。
    float left_pwm = speed_to_open_loop_pwm(left_target);
    float right_pwm = speed_to_open_loop_pwm(right_target);
#endif

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

#if CAR_ENABLE_MOTOR_OUTPUT
    bsp_motor_set(cmd);
#else
    // 电机输出关闭时仍下发 0 PWM，保证切换阶段不会残留上一次命令。
    motor_cmd_t safe_cmd = { .left_pwm = 0.0f, .right_pwm = 0.0f, .left_dir = true, .right_dir = true };
    bsp_motor_set(safe_cmd);
    cmd = safe_cmd;
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
#if CAR_ENABLE_OLED
    if (g_rt_valid) bsp_oled_show_runtime(g_mode, &g_rt);
#endif

#if CAR_ENABLE_PID_SCOPE
    pid_scope_poll_10ms();
#endif
}
