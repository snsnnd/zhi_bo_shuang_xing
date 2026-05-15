#include "bsp_imu.h"
#include "../Common/car_config.h"
#include "i2c.h"

static imu_state_t g_imu;
static float g_dt = 0.01f;
static float g_gyro_z_bias_dps;
static uint32_t g_prev_tick;
static bool g_mpu_ready;

static float lpf(float prev, float in, float a) { return prev + a * (in - prev); }
static float absf(float x) { return x < 0.0f ? -x : x; }
static int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

static bool mpu_write(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, 10U) == HAL_OK;
}

static bool mpu_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10U) == HAL_OK;
}

static bool read_mpu_raw(int16_t *acc_x, int16_t *acc_y, int16_t *gyro_z) {
    uint8_t buf[14];
    if (!mpu_read(0x3BU, buf, sizeof(buf))) return false;
    *acc_x = be16(&buf[0]);
    *acc_y = be16(&buf[2]);
    *gyro_z = be16(&buf[12]);
    return true;
}

static void calibrate_gyro_z(void) {
    int32_t sum = 0;
    uint16_t ok_count = 0U;

    // 上电零漂校准要求车辆静止；失败时保留手动偏置，系统仍可运行。
    for (uint16_t i = 0U; i < MPU6050_GYRO_CALIB_SAMPLES; ++i) {
        int16_t ax, ay, gz;
        if (read_mpu_raw(&ax, &ay, &gz)) {
            (void)ax;
            (void)ay;
            sum += gz;
            ok_count++;
        }
        HAL_Delay(MPU6050_GYRO_CALIB_DELAY_MS);
    }

    if (ok_count > 0U) {
        g_gyro_z_bias_dps = ((float)sum / (float)ok_count) / 131.0f;
    }
}

static void ensure_mpu_ready(void) {
    if (!g_mpu_ready) {
        g_mpu_ready = mpu_write(0x6BU, 0x00U) && mpu_write(0x1BU, 0x00U) && mpu_write(0x1CU, 0x00U);
        if (g_mpu_ready) calibrate_gyro_z();
        g_prev_tick = HAL_GetTick();
    }
}

// IMU 初始化：清空姿态与角速度状态
void bsp_imu_init(float dt_s) {
    g_dt = dt_s;
    g_imu.yaw_deg = 0.0f;
    g_imu.gyro_z_dps = 0.0f;
    g_imu.yaw_rate_filtered = 0.0f;
    g_imu.pitch_deg = 0.0f;
    g_imu.roll_deg = 0.0f;
    g_gyro_z_bias_dps = MPU6050_GYRO_Z_OFFSET_DPS;
    g_prev_tick = HAL_GetTick();
    g_mpu_ready = false;
    ensure_mpu_ready();
}

// IMU 更新：gyro 低通后积分 yaw，pitch/roll 走互补通道
void bsp_imu_update(float gyro_z_raw_dps, float acc_x_g, float acc_y_g) {
    // 保存原始角速度（便于调试与标定）
    g_imu.gyro_z_dps = gyro_z_raw_dps;
    // 对 gyro_z 做低通，抑制高频噪声后再积分 yaw
    g_imu.yaw_rate_filtered = lpf(g_imu.yaw_rate_filtered, gyro_z_raw_dps, IMU_GYRO_LPF_ALPHA);
    g_imu.yaw_deg += g_imu.yaw_rate_filtered * g_dt;

    float pitch_acc_deg = acc_x_g * 57.2958f;
    float roll_acc_deg = acc_y_g * 57.2958f;
    g_imu.pitch_deg = IMU_COMP_ALPHA * g_imu.pitch_deg + (1.0f - IMU_COMP_ALPHA) * pitch_acc_deg;
    g_imu.roll_deg = IMU_COMP_ALPHA * g_imu.roll_deg + (1.0f - IMU_COMP_ALPHA) * roll_acc_deg;

    // 当车辆近似直行且姿态接近平稳时，对 yaw 漂移做微弱慢修正
    if (absf(g_imu.yaw_rate_filtered) < YAW_DRIFT_FIX_RATE_TH_DPS && absf(g_imu.pitch_deg) < YAW_DRIFT_FIX_TILT_TH_DEG && absf(g_imu.roll_deg) < YAW_DRIFT_FIX_TILT_TH_DEG) {
        g_imu.yaw_deg *= YAW_DRIFT_DECAY_WHEN_STRAIGHT;
    }
}

imu_state_t bsp_imu_get_state(void) {
    ensure_mpu_ready();
    if (g_mpu_ready) {
        uint32_t now = HAL_GetTick();
        uint32_t dt_ms = now - g_prev_tick;
        if (dt_ms > 0U) {
            int16_t ax, ay, gz;
            if (read_mpu_raw(&ax, &ay, &gz)) {
                g_dt = (float)dt_ms * 0.001f;
                bsp_imu_update(((float)gz / 131.0f) - g_gyro_z_bias_dps,
                               (float)ax / 16384.0f,
                               (float)ay / 16384.0f);
            }
            g_prev_tick = now;
        }
    }
    return g_imu;
}
void bsp_imu_zero_yaw(void) { g_imu.yaw_deg = 0.0f; }
