#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "../Common/car_types.h"

/*
 * Encoder state abstraction.
 * Speeds are filtered before exposure, while distance is passed through to
 * keep map lookup aligned with real travel distance.
 */
void bsp_encoder_set_state(float left_speed_mps, float right_speed_mps, float distance_m);
encoder_state_t bsp_encoder_get_state(void);

#endif
