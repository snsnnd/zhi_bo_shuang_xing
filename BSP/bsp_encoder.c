#include "bsp_encoder.h"
#include "../Common/car_config.h"

static encoder_state_t g_enc;
static bool g_inited;

static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }

void bsp_encoder_set_state(float left_speed_mps, float right_speed_mps, float distance_m) {
    if (!g_inited) {
        g_enc.left_speed_mps = left_speed_mps;
        g_enc.right_speed_mps = right_speed_mps;
        g_inited = true;
    }
    g_enc.left_speed_mps = lpf(g_enc.left_speed_mps, left_speed_mps, ENC_SPD_LPF_ALPHA);
    g_enc.right_speed_mps = lpf(g_enc.right_speed_mps, right_speed_mps, ENC_SPD_LPF_ALPHA);
    g_enc.distance_m = distance_m;
}

encoder_state_t bsp_encoder_get_state(void) { return g_enc; }
