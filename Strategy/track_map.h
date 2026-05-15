#ifndef TRACK_MAP_H
#define TRACK_MAP_H

#include <stdbool.h>
#include <stdint.h>
#include "../Common/car_types.h"

typedef struct {
    uint16_t id;
    float start_dist_m;
    float end_dist_m;
    segment_type_t type;
    float recommend_speed_mps;
    float feedforward;
    uint8_t risk;
} map_event_t;

typedef struct {
    segment_type_t current_type;
    float start_dist_m;
    float prev_err;
    float yaw_start_deg;
    uint8_t err_sign_flips;
} map_learning_ctx_t;

void track_map_reset(void);
bool track_map_add_event(map_event_t event);
const map_event_t *track_map_find_current(float distance_m);
const map_event_t *track_map_find_next(float distance_m);
uint16_t track_map_count(void);

void track_map_learning_init(map_learning_ctx_t *ctx, float start_dist, float yaw_deg);
bool track_map_learning_step(map_learning_ctx_t *ctx, float distance_m, float yaw_deg, float line_err, line_bin_t bin, bool force_finalize);

#endif
