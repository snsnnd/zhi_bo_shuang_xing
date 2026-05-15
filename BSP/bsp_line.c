#include "bsp_line.h"
#include "../Common/car_config.h"
#include "gpio.h"

static line_raw_t g_filt;
static float g_err_filt;
static bool g_inited;

// 一阶低通滤波：a 越大响应越快，a 越小平滑越强
static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }

static float pin_to_line_value(GPIO_PinState s) {
    bool active = (LINE_SENSOR_ACTIVE_LEVEL != 0U) ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);
    return active ? 1.0f : 0.0f;
}

static void sample_gpio(void) {
    line_raw_t raw = {
        .l2 = pin_to_line_value(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)),
        .l1 = pin_to_line_value(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)),
        .r1 = pin_to_line_value(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)),
        .r2 = pin_to_line_value(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
    };
    bsp_line_set_raw(raw);
}

// 写入一次采样值，并进行轻度低通滤波；GPIO 采样和测试注入共用这条路径。
void bsp_line_set_raw(line_raw_t raw) {
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

static line_bin_t bin_from_filtered(void) {
    line_bin_t b = {
        .l2 = g_filt.l2 > 0.5f,
        .l1 = g_filt.l1 > 0.5f,
        .r1 = g_filt.r1 > 0.5f,
        .r2 = g_filt.r2 > 0.5f
    };
    return b;
}

line_raw_t bsp_line_get_raw(void) {
    sample_gpio();
    return g_filt;
}

line_bin_t bsp_line_get_bin(void) {
    sample_gpio();
    return bin_from_filtered();
}

// 计算加权误差，并对误差再做一次轻滤波降低 PID 抖动
float bsp_line_get_error(void) {
    sample_gpio();
    line_bin_t b = bin_from_filtered();
    float sum = 0.0f, cnt = 0.0f;
    if (b.l2) { sum += LINE_WEIGHT_L2; cnt += 1.0f; }
    if (b.l1) { sum += LINE_WEIGHT_L1; cnt += 1.0f; }
    if (b.r1) { sum += LINE_WEIGHT_R1; cnt += 1.0f; }
    if (b.r2) { sum += LINE_WEIGHT_R2; cnt += 1.0f; }
    float err = (cnt > 0.0f) ? (sum / cnt) : 99.0f;
    g_err_filt = lpf(g_err_filt, err, LINE_ERR_LPF_ALPHA);
    return g_err_filt;
}
