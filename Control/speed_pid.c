#include "speed_pid.h"

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void speed_pid_init(speed_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->dt = dt;
    pid->integral = 0.0f; pid->prev_err = 0.0f;
    pid->out_min = out_min; pid->out_max = out_max;
}

float speed_pid_update(speed_pid_t *pid, float target, float feedback) {
    float err = target - feedback;
    float d = (err - pid->prev_err) / pid->dt;
    pid->integral += err * pid->dt;
    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * d;
    out = clampf(out, pid->out_min, pid->out_max);
    pid->prev_err = err;
    return out;
}

void speed_pid_reset(speed_pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}
