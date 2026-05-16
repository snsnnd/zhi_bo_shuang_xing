#include "bsp_motor.h"
#include "../Common/car_config.h"
#include "gpio.h"
#include "tim.h"

// Last commanded motor output, kept for diagnostics and desktop simulation.
static motor_cmd_t g_cmd;
static bool g_pwm_started;

// 比较最大/小幅度
static float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

// 确保pwm启动
static void ensure_pwm_started(void)
{
    if (!g_pwm_started)
    {
        (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        g_pwm_started = true;
    }
}

// 输出限幅
static uint32_t pwm_to_compare(float pwm)
{
    float limited = clampf(pwm, CAR_MIN_PWM, CAR_MAX_PWM);

    // 线性映射
    float period = (float)__HAL_TIM_GET_AUTORELOAD(&htim4);
    return (uint32_t)((limited / CAR_MAX_PWM) * period);
}

// 设置电机转动方向
static void set_dir(GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin, bool forward, float pwm)
{
    if (pwm <= 0.0f)
    {
        HAL_GPIO_WritePin(a_port, a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(b_port, b_pin, GPIO_PIN_RESET);
    }
    else if (forward)
    {
        HAL_GPIO_WritePin(a_port, a_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(b_port, b_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(a_port, a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(b_port, b_pin, GPIO_PIN_SET);
    }
}

// 设置电机转速
void bsp_motor_set(motor_cmd_t cmd)
{
    ensure_pwm_started();
    cmd.left_pwm = clampf(cmd.left_pwm, CAR_MIN_PWM, CAR_MAX_PWM);
    cmd.right_pwm = clampf(cmd.right_pwm, CAR_MIN_PWM, CAR_MAX_PWM);
    g_cmd = cmd;

    // TB6612 seller reference: A(left)=PB9/TIM4_CH4 + PB14/PB15, B(right)=PB8/TIM4_CH3 + PB13/PB12.
    set_dir(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_14, cmd.left_dir, cmd.left_pwm);
    set_dir(GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, cmd.right_dir, cmd.right_pwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_to_compare(cmd.left_pwm));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_to_compare(cmd.right_pwm));
}

// Return the last command for debug display or unit-level simulation checks.
motor_cmd_t bsp_motor_get_last(void) { return g_cmd; }
