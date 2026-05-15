#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Control-loop timing and PWM limits. */
#define CAR_CTRL_DT_S                 (0.01f)
#define CAR_MAX_PWM                   (1000.0f)
#define CAR_MIN_PWM                   (0.0f)

/* Line sensor weighted-position model: negative is left, positive is right. */
#define LINE_WEIGHT_L2                (-3.0f)
#define LINE_WEIGHT_L1                (-1.0f)
#define LINE_WEIGHT_R1                (1.0f)
#define LINE_WEIGHT_R2                (3.0f)

/* Strategy thresholds for line loss and straight-road prediction. */
#define LINE_LOST_TIMEOUT_MS          (600U)
#define STRAIGHT_ERROR_TH             (0.25f)
#define STRAIGHT_YAW_RATE_TH_DPS      (12.0f)

/* Discrete target speed levels used by strategy and map events. */
#define SPEED_LOW_MPS                 (0.25f)
#define SPEED_MID_LOW_MPS             (0.40f)
#define SPEED_MID_MPS                 (0.55f)
#define SPEED_HIGH_MPS                (0.75f)

/* Look-ahead distances reserved for future map-based prediction tuning. */
#define PREDICT_DIST_LOW_M            (0.08f)
#define PREDICT_DIST_MID_M            (0.20f)
#define PREDICT_DIST_HIGH_M           (0.35f)

/* Maximum number of learned map events persisted in Flash. */
#define MAP_MAX_EVENTS                (64U)

/* Filter and yaw-drift correction coefficients; tune on the real vehicle. */
#define LINE_RAW_LPF_ALPHA            (0.35f)
#define LINE_ERR_LPF_ALPHA            (0.45f)
#define ENC_SPD_LPF_ALPHA             (0.30f)
#define IMU_GYRO_LPF_ALPHA            (0.20f)
#define IMU_COMP_ALPHA                (0.96f)
#define YAW_DRIFT_FIX_RATE_TH_DPS     (6.0f)
#define YAW_DRIFT_FIX_TILT_TH_DEG     (8.0f)
#define YAW_DRIFT_DECAY_WHEN_STRAIGHT (0.9995f)

#endif
