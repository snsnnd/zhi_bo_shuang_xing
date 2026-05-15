#include "line_pid.h"

// Clamp PID output to the actuator range requested by the caller.
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Initialize line PID gains, sample time, output limits, and history state.
void line_pid_init(line_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->dt = dt;
    pid->integral = 0.0f; pid->prev_err = 0.0f;
    pid->out_min = out_min; pid->out_max = out_max;
}

// Convert signed line error into a bounded steering correction.
float line_pid_update(line_pid_t *pid, float err) {
    float d = (err - pid->prev_err) / pid->dt;
    pid->integral += err * pid->dt;
    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * d;
    out = clampf(out, pid->out_min, pid->out_max);
    pid->prev_err = err;
    return out;
}

// Clear integral and derivative history before restarting closed-loop control.
void line_pid_reset(line_pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}
