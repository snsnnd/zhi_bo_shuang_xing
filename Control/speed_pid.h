#ifndef SPEED_PID_H
#define SPEED_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_err;
    float out_min;
    float out_max;
} speed_pid_t;

void speed_pid_init(speed_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max);
float speed_pid_update(speed_pid_t *pid, float target, float feedback);
void speed_pid_reset(speed_pid_t *pid);

#endif
