#include "bsp_encoder.h"

static encoder_state_t g_enc;

void bsp_encoder_set_state(float left_speed_mps, float right_speed_mps, float distance_m) {
    g_enc.left_speed_mps = left_speed_mps;
    g_enc.right_speed_mps = right_speed_mps;
    g_enc.distance_m = distance_m;
}

encoder_state_t bsp_encoder_get_state(void) { return g_enc; }
