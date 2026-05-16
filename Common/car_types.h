#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// Whole-vehicle operating modes shared by application, strategy, and display.
typedef enum
{
    CAR_MODE_IDLE = 0,
    CAR_MODE_CALIBRATION, // 校准模式
    CAR_MODE_LEARNING,    // 学习/建图模式
    CAR_MODE_TRACKING,    // 循线模式
    CAR_MODE_PREDICTION,  // 预测控制模式，在建完图之后
    CAR_MODE_HAIRPIN,     // 极速转弯模式
    CAR_MODE_LOST,        // 丢线模式
    CAR_MODE_PROTECT      // 锁死模式
} car_mode_t;

// Learned track segment categories used by the map and prediction strategy.
typedef enum
{
    SEG_UNKNOWN = 0, // 未知/未识别赛道
    SEG_STRAIGHT,    // 大直道
    SEG_CURVE_LEFT,  // 曲线
    SEG_CURVE_RIGHT,
    SEG_SHARP_LEFT, // 锐角/直角急弯
    SEG_SHARP_RIGHT,
    SEG_S_CURVE,  // S 型蛇形弯
    SEG_U_TURN,   // 发卡弯
    SEG_RISK      // 十字等特殊路段
} segment_type_t; // 赛道段类型

// Normalized four-channel line sensor values in vehicle order: L2/L1/R1/R2 from left to right.
typedef struct
{
    float l2;
    float l1;
    float r1;
    float r2;
} line_raw_t;

// Binary line sensor state after thresholding filtered analog values.
typedef struct
{
    bool l2;
    bool l1;
    bool r1;
    bool r2;
} line_bin_t;

// IMU state consumed by strategy and debugging display.
typedef struct
{
    float yaw_deg;
    float gyro_z_dps; // dps = Degrees Per Second
    float yaw_rate_filtered;
    float pitch_deg;
    float roll_deg;
} imu_state_t;

// Differential encoder-derived speed and accumulated travel distance.
typedef struct
{
    float left_speed_mps; // mps = Meters Per Second
    float right_speed_mps;
    float distance_m;
} encoder_state_t;

// Motor command for a two-wheel differential chassis.
typedef struct
{
    float left_pwm;
    float right_pwm;
    bool left_dir;
    bool right_dir;
} motor_cmd_t;

#endif
