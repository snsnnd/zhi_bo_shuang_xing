#include "bsp_line.h"
#include "../Common/car_config.h"

static line_raw_t g_raw;

void bsp_line_set_raw(line_raw_t raw) { g_raw = raw; }
line_raw_t bsp_line_get_raw(void) { return g_raw; }

line_bin_t bsp_line_get_bin(void) {
    line_bin_t b = {
        .l2 = g_raw.l2 > 0.5f,
        .l1 = g_raw.l1 > 0.5f,
        .r1 = g_raw.r1 > 0.5f,
        .r2 = g_raw.r2 > 0.5f
    };
    return b;
}

float bsp_line_get_error(void) {
    line_bin_t b = bsp_line_get_bin();
    float sum = 0.0f, cnt = 0.0f;
    if (b.l2) { sum += LINE_WEIGHT_L2; cnt += 1.0f; }
    if (b.l1) { sum += LINE_WEIGHT_L1; cnt += 1.0f; }
    if (b.r1) { sum += LINE_WEIGHT_R1; cnt += 1.0f; }
    if (b.r2) { sum += LINE_WEIGHT_R2; cnt += 1.0f; }
    return (cnt > 0.0f) ? (sum / cnt) : 99.0f;
}
