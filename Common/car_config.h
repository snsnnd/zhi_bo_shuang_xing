#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define CAR_CTRL_DT_S                 (0.01f)
#define CAR_MAX_PWM                   (1000.0f)
#define CAR_MIN_PWM                   (0.0f)

#define LINE_WEIGHT_L2                (-3.0f)
#define LINE_WEIGHT_L1                (-1.0f)
#define LINE_WEIGHT_R1                (1.0f)
#define LINE_WEIGHT_R2                (3.0f)

#define LINE_LOST_TIMEOUT_MS          (600U)
#define STRAIGHT_ERROR_TH             (0.25f)
#define STRAIGHT_YAW_RATE_TH_DPS      (12.0f)

#define SPEED_LOW_MPS                 (0.25f)
#define SPEED_MID_LOW_MPS             (0.40f)
#define SPEED_MID_MPS                 (0.55f)
#define SPEED_HIGH_MPS                (0.75f)

#define PREDICT_DIST_LOW_M            (0.08f)
#define PREDICT_DIST_MID_M            (0.20f)
#define PREDICT_DIST_HIGH_M           (0.35f)

#define MAP_MAX_EVENTS                (64U)

#endif
