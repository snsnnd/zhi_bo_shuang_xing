#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "../Common/car_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*remote_write_fn)(const char *text);
typedef uint32_t (*remote_time_fn)(void);

typedef enum {
    REMOTE_PID_NONE = 0,
    REMOTE_PID_LINE,
    REMOTE_PID_SPEED_LEFT,
    REMOTE_PID_SPEED_RIGHT,
    REMOTE_PID_SPEED_BOTH
} remote_pid_target_t;

typedef enum {
    REMOTE_MOTOR_TEST_NONE = 0,
    REMOTE_MOTOR_TEST_LEFT,
    REMOTE_MOTOR_TEST_RIGHT,
    REMOTE_MOTOR_TEST_BOTH
} remote_motor_test_mode_t;

typedef enum {
    REMOTE_SPEED_TUNE_NONE = 0,
    REMOTE_SPEED_TUNE_LEFT,
    REMOTE_SPEED_TUNE_RIGHT,
    REMOTE_SPEED_TUNE_BOTH
} remote_speed_tune_mode_t;

typedef struct {
    remote_pid_target_t target;
    float kp;
    float ki;
    float kd;
    bool pending;
} remote_pid_update_t;

typedef struct {
    bool motor_enable;
    bool force_stop;
    bool speed_override;
    bool mode_override;
    bool speed_limit_override;
    float target_speed_mps;
    float speed_limit_mps;
    car_mode_t mode;
    remote_pid_update_t pid_update;
    remote_motor_test_mode_t motor_test_mode;
    remote_speed_tune_mode_t speed_tune_mode;
    float motor_test_pwm;
    bool encoder_query_pending;
    uint8_t scope_device_id;
    uint8_t scope_channel_id;
    uint32_t last_cmd_ms;
    uint32_t timeout_ms;
} remote_control_state_t;

typedef struct {
    remote_write_fn write_text;
    remote_time_fn get_time_ms;
} remote_control_port_t;

void remote_control_init(const remote_control_port_t *port);
void remote_control_rx_byte(uint8_t byte);
void remote_control_poll(void);
remote_control_state_t remote_control_get_state(void);
void remote_control_clear_pid_update(void);
void remote_control_reply_encoder(float left_speed_mps, float right_speed_mps, float distance_m);

#ifdef __cplusplus
}
#endif

#endif
