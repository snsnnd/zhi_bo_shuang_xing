#ifndef SPEED_PID_H
#define SPEED_PID_H

// PID controller state for one wheel speed loop.
typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_err;
    float out_min;
    float out_max;
    float last_target;
    float last_feedback;
    float last_error;
    float last_output;
    float last_p_term;
    float last_i_term;
    float last_d_term;
} speed_pid_t;

/* Fixed-period speed PID API. */
void speed_pid_init(speed_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max);
float speed_pid_update(speed_pid_t *pid, float target, float feedback);
void speed_pid_reset(speed_pid_t *pid);

#endif
