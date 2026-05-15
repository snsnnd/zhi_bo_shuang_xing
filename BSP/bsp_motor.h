#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "../Common/car_types.h"

void bsp_motor_set(motor_cmd_t cmd);
motor_cmd_t bsp_motor_get_last(void);

#endif
