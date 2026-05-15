#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "../Common/car_types.h"

void bsp_imu_init(float dt_s);
void bsp_imu_update(float gyro_z_raw_dps, float acc_x_g, float acc_y_g);
imu_state_t bsp_imu_get_state(void);
void bsp_imu_zero_yaw(void);

#endif
