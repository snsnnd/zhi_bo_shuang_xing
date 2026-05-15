#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "../Common/car_types.h"

/*
 * Motor output abstraction.
 * Current implementation stores the command as a mock; hardware integration
 * should map left/right PWM and direction to TIM4 PWM plus GPIO direction pins.
 */
void bsp_motor_set(motor_cmd_t cmd);
motor_cmd_t bsp_motor_get_last(void);

#endif
