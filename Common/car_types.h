#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// Whole-vehicle operating modes shared by application, strategy, and display.
typedef enum {
    CAR_MODE_IDLE = 0,
    CAR_MODE_CALIBRATION,
    CAR_MODE_LEARNING,
    CAR_MODE_TRACKING,
    CAR_MODE_PREDICTION,
    CAR_MODE_HAIRPIN,
    CAR_MODE_LOST,
    CAR_MODE_PROTECT
} car_mode_t;

// Learned track segment categories used by the map and prediction strategy.
typedef enum {
    SEG_UNKNOWN = 0,
    SEG_STRAIGHT,
    SEG_CURVE_LEFT,
    SEG_CURVE_RIGHT,
    SEG_SHARP_LEFT,
    SEG_SHARP_RIGHT,
    SEG_S_CURVE,
    SEG_U_TURN,
    SEG_RISK
} segment_type_t;

// Normalized four-channel line sensor values: L2/L1/R1/R2 from left to right.
typedef struct {
    float l2;
    float l1;
    float r1;
    float r2;
} line_raw_t;

// Binary line sensor state after thresholding filtered analog values.
typedef struct {
    bool l2;
    bool l1;
    bool r1;
    bool r2;
} line_bin_t;

// IMU state consumed by strategy and debugging display.
typedef struct {
    float yaw_deg;
    float gyro_z_dps;
    float yaw_rate_filtered;
    float pitch_deg;
    float roll_deg;
} imu_state_t;

// Differential encoder-derived speed and accumulated travel distance.
typedef struct {
    float left_speed_mps;
    float right_speed_mps;
    float distance_m;
} encoder_state_t;

// Motor command for a two-wheel differential chassis.
typedef struct {
    float left_pwm;
    float right_pwm;
    bool left_dir;
    bool right_dir;
} motor_cmd_t;

#endif
