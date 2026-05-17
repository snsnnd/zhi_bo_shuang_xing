#include "bsp_encoder.h"
#include "../Common/car_config.h"
#include "tim.h"

static encoder_state_t g_enc;
static bool g_inited;
static bool g_hw_started; // 底层硬件状态
static uint16_t g_prev_lcnt;
static uint16_t g_prev_rcnt;
static uint32_t g_prev_tick;

// 一阶低通滤波
static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }

// 判断编码器是否是第一次启动
static void ensure_encoder_started(void)
{
    if (!g_hw_started) // 硬件是否启动
    {
        (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
        (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        g_prev_lcnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
        g_prev_rcnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        g_prev_tick = HAL_GetTick();
        g_hw_started = true;
    }
}

// 更新编码器状态：速度低通，累计距离直通（避免地图定位滞后）
void bsp_encoder_set_state(float left_speed_mps, float right_speed_mps, float distance_m)
{
    if (!g_inited)
    {
        g_enc.left_speed_mps = left_speed_mps;
        g_enc.right_speed_mps = right_speed_mps;
        g_inited = true;
    }
    g_enc.left_speed_mps = lpf(g_enc.left_speed_mps, left_speed_mps, ENC_SPD_LPF_ALPHA);
    g_enc.right_speed_mps = lpf(g_enc.right_speed_mps, right_speed_mps, ENC_SPD_LPF_ALPHA);
    g_enc.distance_m = distance_m;
}

// 获取编码器状态
encoder_state_t bsp_encoder_get_state(void)
{
    ensure_encoder_started();

    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - g_prev_tick;
    if (dt_ms == 0U)
        return g_enc;

    uint16_t tim2_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t tim3_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t tim2_delta = (int16_t)(tim2_cnt - g_prev_lcnt);
    int16_t tim3_delta = (int16_t)(tim3_cnt - g_prev_rcnt);
    g_prev_lcnt = tim2_cnt;
    g_prev_rcnt = tim3_cnt;
    g_prev_tick = now;

#if ENCODER_SWAP_LEFT_RIGHT
    int16_t ldelta = tim3_delta;
    int16_t rdelta = tim2_delta;
#else
    int16_t ldelta = tim2_delta;
    int16_t rdelta = tim3_delta;
#endif
    ldelta = (int16_t)(ldelta * ENCODER_LEFT_SIGN);
    rdelta = (int16_t)(rdelta * ENCODER_RIGHT_SIGN);

    float dt_s = (float)dt_ms * 0.001f;
    float wheel_circ_m = WHEEL_DIAMETER_M * 3.1415926f;
    float left_dist = ((float)ldelta / ENCODER_COUNTS_PER_REV) * wheel_circ_m;
    float right_dist = ((float)rdelta / ENCODER_COUNTS_PER_REV) * wheel_circ_m;
    float left_speed = left_dist / dt_s;
    float right_speed = right_dist / dt_s;

    if (!g_inited)
    {
        g_enc.left_speed_mps = left_speed;
        g_enc.right_speed_mps = right_speed;
        g_inited = true;
    }
    else
    {
        g_enc.left_speed_mps = lpf(g_enc.left_speed_mps, left_speed, ENC_SPD_LPF_ALPHA);
        g_enc.right_speed_mps = lpf(g_enc.right_speed_mps, right_speed, ENC_SPD_LPF_ALPHA);
    }
    g_enc.distance_m += (left_dist + right_dist) * 0.5f;
    return g_enc;
}
