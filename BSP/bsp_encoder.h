#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "../Common/car_types.h"

void bsp_encoder_set_state(float left_speed_mps, float right_speed_mps, float distance_m);
encoder_state_t bsp_encoder_get_state(void);

#endif
