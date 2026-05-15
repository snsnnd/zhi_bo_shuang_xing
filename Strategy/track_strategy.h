#ifndef TRACK_STRATEGY_H
#define TRACK_STRATEGY_H

#include "../Common/car_types.h"
#include "track_map.h"

typedef struct {
    car_mode_t mode;
    float target_speed_mps;
    float steer_limit;
    bool lost_timeout;
} strategy_output_t;

void track_strategy_init(void);
strategy_output_t track_strategy_step(float line_error, line_bin_t bin, imu_state_t imu, encoder_state_t enc, uint32_t tick_ms, const map_event_t *current_event);

#endif
