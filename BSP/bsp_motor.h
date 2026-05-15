#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "../Common/car_types.h"

/*
 * Motor output abstraction.
 * Maps left/right PWM to TIM4 CH3/CH4 and direction to PB12-PB15.
 */
void bsp_motor_set(motor_cmd_t cmd);
motor_cmd_t bsp_motor_get_last(void);

#endif
