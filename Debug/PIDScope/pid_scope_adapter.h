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
void pid_scope_poll_10ms(void);
void pid_scope_rx_byte(uint8_t byte);

/* Implement this in the USART driver after CubeMX enables a serial port. */
int pid_scope_port_write(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
