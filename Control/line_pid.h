#ifndef LINE_PID_H
#define LINE_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_err;
    float out_min;
    float out_max;
} line_pid_t;

void line_pid_init(line_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max);
float line_pid_update(line_pid_t *pid, float err);
void line_pid_reset(line_pid_t *pid);

#endif
