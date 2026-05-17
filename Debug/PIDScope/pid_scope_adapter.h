#ifndef PID_SCOPE_ADAPTER_H
#define PID_SCOPE_ADAPTER_H

#include "../../Control/line_pid.h"
#include "../../Control/speed_pid.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pid_scope_init(void);
void pid_scope_bind_line_pid(line_pid_t *pid);
void pid_scope_bind_left_speed_pid(speed_pid_t *pid);
void pid_scope_bind_right_speed_pid(speed_pid_t *pid);
void pid_scope_set_selected_channel(uint8_t device_id, uint8_t channel_id);
void pid_scope_set_send_period_ms(uint32_t period_ms);
uint32_t pid_scope_get_send_period_ms(void);
void pid_scope_set_vehicle_state(float distance_m, float yaw_deg, uint16_t status_flags);
void pid_scope_set_map_state(uint16_t current_id, uint8_t current_type,
                             uint16_t next_id, uint8_t next_type,
                             float dist_to_next_m, uint8_t car_mode);
void pid_scope_poll_10ms(void);
void pid_scope_rx_byte(uint8_t byte);

/* Implement this in the USART driver after CubeMX enables a serial port. */
int pid_scope_port_write(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
