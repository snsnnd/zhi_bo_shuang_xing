#include "bsp_line.h"
#include "../Common/car_config.h"

static line_raw_t g_raw;
static line_raw_t g_filt;
static float g_err_filt;
static bool g_inited;

// 一阶低通滤波：a 越大响应越快，a 越小平滑越强
static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }

// 写入原始循迹值，并进行轻度低通滤波
void bsp_line_set_raw(line_raw_t raw) {
    g_raw = raw;
    if (!g_inited) {
        g_filt = raw;
        g_err_filt = 0.0f;
        g_inited = true;
    }

    g_filt.l2 = lpf(g_filt.l2, raw.l2, LINE_RAW_LPF_ALPHA);
    g_filt.l1 = lpf(g_filt.l1, raw.l1, LINE_RAW_LPF_ALPHA);
    g_filt.r1 = lpf(g_filt.r1, raw.r1, LINE_RAW_LPF_ALPHA);
    g_filt.r2 = lpf(g_filt.r2, raw.r2, LINE_RAW_LPF_ALPHA);
}

line_raw_t bsp_line_get_raw(void) { return g_filt; }

line_bin_t bsp_line_get_bin(void) {
    line_bin_t b = {
        .l2 = g_filt.l2 > 0.5f,
        .l1 = g_filt.l1 > 0.5f,
        .r1 = g_filt.r1 > 0.5f,
        .r2 = g_filt.r2 > 0.5f
    };
    return b;
}

// 计算加权误差，并对误差再做一次轻滤波降低 PID 抖动
float bsp_line_get_error(void) {
    line_bin_t b = bsp_line_get_bin();
    float sum = 0.0f, cnt = 0.0f;
    if (b.l2) { sum += LINE_WEIGHT_L2; cnt += 1.0f; }
    if (b.l1) { sum += LINE_WEIGHT_L1; cnt += 1.0f; }
    if (b.r1) { sum += LINE_WEIGHT_R1; cnt += 1.0f; }
    if (b.r2) { sum += LINE_WEIGHT_R2; cnt += 1.0f; }
    float err = (cnt > 0.0f) ? (sum / cnt) : 99.0f;
    g_err_filt = lpf(g_err_filt, err, LINE_ERR_LPF_ALPHA);
    return g_err_filt;
}
