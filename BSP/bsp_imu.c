#include "bsp_imu.h"
#include "../Common/car_config.h"

static imu_state_t g_imu;
static float g_dt = 0.01f;

static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }
static float absf(float x) { return x < 0.0f ? -x : x; }

void bsp_imu_init(float dt_s) {
    g_dt = dt_s;
    g_imu.yaw_deg = 0.0f;
    g_imu.gyro_z_dps = 0.0f;
    g_imu.yaw_rate_filtered = 0.0f;
    g_imu.pitch_deg = 0.0f;
    g_imu.roll_deg = 0.0f;
}

void bsp_imu_update(float gyro_z_raw_dps, float acc_x_g, float acc_y_g) {
    g_imu.gyro_z_dps = gyro_z_raw_dps;
    g_imu.yaw_rate_filtered = lpf(g_imu.yaw_rate_filtered, gyro_z_raw_dps, IMU_GYRO_LPF_ALPHA);
    g_imu.yaw_deg += g_imu.yaw_rate_filtered * g_dt;

    float pitch_acc_deg = acc_x_g * 57.2958f;
    float roll_acc_deg = acc_y_g * 57.2958f;
    g_imu.pitch_deg = IMU_COMP_ALPHA * g_imu.pitch_deg + (1.0f - IMU_COMP_ALPHA) * pitch_acc_deg;
    g_imu.roll_deg = IMU_COMP_ALPHA * g_imu.roll_deg + (1.0f - IMU_COMP_ALPHA) * roll_acc_deg;

    if (absf(g_imu.yaw_rate_filtered) < YAW_DRIFT_FIX_RATE_TH_DPS && absf(g_imu.pitch_deg) < YAW_DRIFT_FIX_TILT_TH_DEG && absf(g_imu.roll_deg) < YAW_DRIFT_FIX_TILT_TH_DEG) {
        g_imu.yaw_deg *= YAW_DRIFT_DECAY_WHEN_STRAIGHT;
    }
}

imu_state_t bsp_imu_get_state(void) { return g_imu; }
void bsp_imu_zero_yaw(void) { g_imu.yaw_deg = 0.0f; }
