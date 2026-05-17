#include "pid_scope_adapter.h"

#include "../../Common/car_config.h"
#include "../../Common/runtime_config.h"
#if CAR_ENABLE_PID_SCOPE_OVER_HC05
#include "../../BSP/bsp_hc05.h"
#endif
#include "pid_debug.h"
#include "main.h"

static line_pid_t *g_line_pid;
static speed_pid_t *g_left_speed_pid;
static speed_pid_t *g_right_speed_pid;
static uint32_t g_last_send_ms;
static uint32_t g_last_heartbeat_ms;
static uint32_t g_last_map_status_ms;
static uint32_t g_send_period_ms = PID_SCOPE_SEND_PERIOD_MS;
static uint8_t g_selected_device_id = PID_SCOPE_DEVICE_ID;
static uint8_t g_selected_channel_id = PID_SCOPE_CH_SPEED_LEFT;
static uint8_t g_speed_both_next_channel = PID_SCOPE_CH_SPEED_LEFT;
static float g_vehicle_distance_m;
static float g_vehicle_yaw_deg;
static uint16_t g_vehicle_status_flags;
static uint16_t g_current_map_id = 0xFFFFU;
static uint8_t g_current_map_type;
static uint16_t g_next_map_id = 0xFFFFU;
static uint8_t g_next_map_type;
static float g_dist_to_next_m = -1.0f;
static uint8_t g_car_mode;

__weak int pid_scope_port_write(const uint8_t *data, uint16_t len) {
#if CAR_ENABLE_PID_SCOPE_OVER_HC05
    if (!runtime_feature_enabled(RC_FEATURE_PID_SCOPE) || !runtime_feature_enabled(RC_FEATURE_PID_SCOPE_OVER_HC05)) return 0;
    return bsp_hc05_send(data, len);
#else
    (void)data;
    (void)len;
    return 0;
#endif
}

static uint32_t debug_time_ms(void) { return HAL_GetTick(); }
static int debug_write(const uint8_t *data, uint16_t len) { return pid_scope_port_write(data, len); }

static void apply_line_param(const pid_debug_param_t *param) {
    if (!g_line_pid || !param) return;
    g_line_pid->kp = param->kp;
    g_line_pid->ki = param->enable_integral ? param->ki : 0.0f;
    g_line_pid->kd = param->enable_derivative ? param->kd : 0.0f;
    g_line_pid->out_min = param->output_min;
    g_line_pid->out_max = param->output_max;
    line_pid_reset(g_line_pid);
}

static void apply_speed_param(speed_pid_t *pid, const pid_debug_param_t *param) {
    if (!pid || !param) return;
    pid->kp = param->kp;
    pid->ki = param->enable_integral ? param->ki : 0.0f;
    pid->kd = param->enable_derivative ? param->kd : 0.0f;
    pid->out_min = param->output_min;
    pid->out_max = param->output_max;
    speed_pid_reset(pid);
}

static pid_debug_param_t param_from_line(const line_pid_t *pid) {
    pid_debug_param_t p = {0};
    p.kp = pid->kp; p.ki = pid->ki; p.kd = pid->kd;
    p.output_min = pid->out_min; p.output_max = pid->out_max;
    p.integral_min = -10000.0f; p.integral_max = 10000.0f;
    p.d_filter_alpha = 0.15f;
    p.enable_integral = pid->ki != 0.0f;
    p.enable_derivative = pid->kd != 0.0f;
    p.enable_anti_windup = 1U;
    return p;
}

static pid_debug_param_t param_from_speed(const speed_pid_t *pid) {
    pid_debug_param_t p = {0};
    p.kp = pid->kp; p.ki = pid->ki; p.kd = pid->kd;
    p.output_min = pid->out_min; p.output_max = pid->out_max;
    p.integral_min = -10000.0f; p.integral_max = 10000.0f;
    p.d_filter_alpha = 0.15f;
    p.enable_integral = pid->ki != 0.0f;
    p.enable_derivative = pid->kd != 0.0f;
    p.enable_anti_windup = 1U;
    return p;
}

static void on_param_set(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param) {
    if (device_id != PID_SCOPE_DEVICE_ID) return;
    if (channel_id == PID_SCOPE_CH_LINE) apply_line_param(param);
    else if (channel_id == PID_SCOPE_CH_SPEED_LEFT) apply_speed_param(g_left_speed_pid, param);
    else if (channel_id == PID_SCOPE_CH_SPEED_RIGHT) apply_speed_param(g_right_speed_pid, param);
}

static void send_line_telemetry(void) {
    if (!g_line_pid) return;
    pid_debug_telemetry_t t = {0};
    t.timestamp_ms = HAL_GetTick();
    t.target = g_line_pid->last_target;
    t.feedback = g_line_pid->last_feedback;
    t.error = g_line_pid->last_error;
    t.output = g_line_pid->last_output;
    t.kp = g_line_pid->kp; t.ki = g_line_pid->ki; t.kd = g_line_pid->kd;
    t.p_term = g_line_pid->last_p_term;
    t.i_term = g_line_pid->last_i_term;
    t.d_term = g_line_pid->last_d_term;
    t.extra1 = g_vehicle_distance_m;
    t.extra2 = g_vehicle_yaw_deg;
    t.status_flags = g_vehicle_status_flags;
    (void)pid_debug_send_telemetry(PID_SCOPE_DEVICE_ID, PID_SCOPE_CH_LINE, &t);
}

static void send_speed_telemetry(uint8_t channel_id, const speed_pid_t *pid) {
    if (!pid) return;
    pid_debug_telemetry_t t = {0};
    t.timestamp_ms = HAL_GetTick();
    t.target = pid->last_target;
    t.feedback = pid->last_feedback;
    t.error = pid->last_error;
    t.output = pid->last_output;
    t.kp = pid->kp; t.ki = pid->ki; t.kd = pid->kd;
    t.p_term = pid->last_p_term;
    t.i_term = pid->last_i_term;
    t.d_term = pid->last_d_term;
    t.extra1 = g_vehicle_distance_m;
    t.extra2 = g_vehicle_yaw_deg;
    t.status_flags = g_vehicle_status_flags;
    (void)pid_debug_send_telemetry(PID_SCOPE_DEVICE_ID, channel_id, &t);
}

void pid_scope_init(void) {
    pid_debug_port_t port = { debug_write, debug_time_ms, on_param_set };
    pid_debug_init(&port);
}

void pid_scope_bind_line_pid(line_pid_t *pid) {
    g_line_pid = pid;
    if (pid) {
        pid_debug_param_t p = param_from_line(pid);
        (void)pid_debug_send_param_report(PID_SCOPE_DEVICE_ID, PID_SCOPE_CH_LINE, &p);
    }
}

void pid_scope_bind_left_speed_pid(speed_pid_t *pid) {
    g_left_speed_pid = pid;
    if (pid) {
        pid_debug_param_t p = param_from_speed(pid);
        (void)pid_debug_send_param_report(PID_SCOPE_DEVICE_ID, PID_SCOPE_CH_SPEED_LEFT, &p);
    }
}

void pid_scope_bind_right_speed_pid(speed_pid_t *pid) {
    g_right_speed_pid = pid;
    if (pid) {
        pid_debug_param_t p = param_from_speed(pid);
        (void)pid_debug_send_param_report(PID_SCOPE_DEVICE_ID, PID_SCOPE_CH_SPEED_RIGHT, &p);
    }
}

void pid_scope_set_selected_channel(uint8_t device_id, uint8_t channel_id) {
    g_selected_device_id = device_id;
    g_selected_channel_id = channel_id;
}

void pid_scope_set_send_period_ms(uint32_t period_ms) {
    if (period_ms < PID_SCOPE_SEND_PERIOD_MIN_MS) period_ms = PID_SCOPE_SEND_PERIOD_MIN_MS;
    if (period_ms > PID_SCOPE_SEND_PERIOD_MAX_MS) period_ms = PID_SCOPE_SEND_PERIOD_MAX_MS;
    g_send_period_ms = period_ms;
}

uint32_t pid_scope_get_send_period_ms(void) { return g_send_period_ms; }

void pid_scope_set_vehicle_state(float distance_m, float yaw_deg, uint16_t status_flags) {
    g_vehicle_distance_m = distance_m;
    g_vehicle_yaw_deg = yaw_deg;
    g_vehicle_status_flags = status_flags;
}

void pid_scope_set_map_state(uint16_t current_id, uint8_t current_type,
                             uint16_t next_id, uint8_t next_type,
                             float dist_to_next_m, uint8_t car_mode) {
    g_current_map_id = current_id;
    g_current_map_type = current_type;
    g_next_map_id = next_id;
    g_next_map_type = next_type;
    g_dist_to_next_m = dist_to_next_m;
    g_car_mode = car_mode;
}

void pid_scope_poll_10ms(void) {
    uint32_t now = HAL_GetTick();
    if ((now - g_last_heartbeat_ms) >= 1000U) {
        g_last_heartbeat_ms = now;
        (void)pid_debug_send_heartbeat(PID_SCOPE_DEVICE_ID, g_vehicle_status_flags, now);
    }
    if ((now - g_last_map_status_ms) >= 500U) {
        g_last_map_status_ms = now;
        (void)pid_debug_send_map_status(PID_SCOPE_DEVICE_ID, now,
                                        g_vehicle_distance_m, g_vehicle_yaw_deg,
                                        g_current_map_id, g_current_map_type,
                                        g_next_map_id, g_next_map_type,
                                        g_dist_to_next_m, g_car_mode,
                                        g_vehicle_status_flags);
    }
    if ((now - g_last_send_ms) >= g_send_period_ms) {
        g_last_send_ms = now;
        if (g_selected_device_id == PID_SCOPE_DEVICE_ID) {
            if (g_selected_channel_id == PID_SCOPE_CH_LINE) send_line_telemetry();
            else if (g_selected_channel_id == PID_SCOPE_CH_SPEED_LEFT) send_speed_telemetry(PID_SCOPE_CH_SPEED_LEFT, g_left_speed_pid);
            else if (g_selected_channel_id == PID_SCOPE_CH_SPEED_RIGHT) send_speed_telemetry(PID_SCOPE_CH_SPEED_RIGHT, g_right_speed_pid);
            else if (g_selected_channel_id == PID_SCOPE_CH_SPEED_BOTH) {
                if (g_speed_both_next_channel == PID_SCOPE_CH_SPEED_LEFT) {
                    send_speed_telemetry(PID_SCOPE_CH_SPEED_LEFT, g_left_speed_pid);
                    g_speed_both_next_channel = PID_SCOPE_CH_SPEED_RIGHT;
                } else {
                    send_speed_telemetry(PID_SCOPE_CH_SPEED_RIGHT, g_right_speed_pid);
                    g_speed_both_next_channel = PID_SCOPE_CH_SPEED_LEFT;
                }
            }
        }
    }
    pid_debug_poll();
}

void pid_scope_rx_byte(uint8_t byte) { pid_debug_rx_byte(byte); }
