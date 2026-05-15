#include "bsp_imu.h"

static imu_state_t g_imu;
static float g_dt = 0.01f;

void bsp_imu_init(float dt_s) {
    g_dt = dt_s;
    g_imu.yaw_deg = 0.0f;
    g_imu.gyro_z_dps = 0.0f;
    g_imu.yaw_rate_filtered = 0.0f;
}

void bsp_imu_update(float gyro_z_raw_dps, float acc_x_g, float acc_y_g) {
    (void)acc_x_g;
    (void)acc_y_g;
    const float lpf_alpha = 0.20f;
    g_imu.gyro_z_dps = gyro_z_raw_dps;
    g_imu.yaw_rate_filtered += lpf_alpha * (gyro_z_raw_dps - g_imu.yaw_rate_filtered);
    g_imu.yaw_deg += g_imu.yaw_rate_filtered * g_dt;
}

imu_state_t bsp_imu_get_state(void) { return g_imu; }
void bsp_imu_zero_yaw(void) { g_imu.yaw_deg = 0.0f; }
