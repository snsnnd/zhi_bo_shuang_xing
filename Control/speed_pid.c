#include "speed_pid.h"

// Clamp PWM output to the configured motor command range.
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Initialize speed PID gains, sample time, output limits, and history state.
void speed_pid_init(speed_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->dt = dt;
    pid->integral = 0.0f; pid->prev_err = 0.0f;
    pid->out_min = out_min; pid->out_max = out_max;
    pid->last_target = 0.0f; pid->last_feedback = 0.0f; pid->last_error = 0.0f; pid->last_output = 0.0f;
    pid->last_p_term = 0.0f; pid->last_i_term = 0.0f; pid->last_d_term = 0.0f;
}

// Convert target/feedback speed error into a bounded PWM command.
float speed_pid_update(speed_pid_t *pid, float target, float feedback) {
    float err = target - feedback;
    float d = (err - pid->prev_err) / pid->dt;
    pid->integral += err * pid->dt;
    pid->last_target = target;
    pid->last_feedback = feedback;
    pid->last_error = err;
    pid->last_p_term = pid->kp * err;
    pid->last_i_term = pid->ki * pid->integral;
    pid->last_d_term = pid->kd * d;
    float out = pid->last_p_term + pid->last_i_term + pid->last_d_term;
    out = clampf(out, pid->out_min, pid->out_max);
    pid->last_output = out;
    pid->prev_err = err;
    return out;
}

// Clear integral and derivative history before restarting speed control.
void speed_pid_reset(speed_pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    pid->last_p_term = 0.0f;
    pid->last_i_term = 0.0f;
    pid->last_d_term = 0.0f;
}
