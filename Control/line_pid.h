#ifndef LINE_PID_H
#define LINE_PID_H

// PID controller state for converting line-position error to steering output.
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
} line_pid_t;

/* Fixed-period line PID API. */
void line_pid_init(line_pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max);
float line_pid_update(line_pid_t *pid, float err);
void line_pid_reset(line_pid_t *pid);

#endif
