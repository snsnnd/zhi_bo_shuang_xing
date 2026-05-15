# STM32 智能车循迹系统（含 Flash 地图持久化 + 分层滤波融合）

## 本次新增：按你的建议落地滤波组合

- 四路循迹原始值：一阶低通滤波（轻滤波，低延迟）
- 循迹误差 error：轻度低通滤波，降低 PID 抖动
- 编码器速度：一阶低通滤波（累计距离不滤波）
- 陀螺仪 gyro_z：低通后积分 yaw
- pitch/roll：加入简化互补滤波通道
- yaw：不做“accel-gyro硬互补修正”，改用工程慢修正（低角速且近水平时轻微衰减漂移）

> 这符合 MPU6050 无磁力计的实际约束：yaw 不能靠标准互补滤波长期锁定。

## 关键实现点

- `BSP/bsp_line.c`
  - `bsp_line_set_raw()` 对 L2/L1/R1/R2 做 LPF
  - `bsp_line_get_error()` 对误差再做轻滤波

- `BSP/bsp_encoder.c`
  - 左右轮速度 LPF
  - 距离 `distance_m` 直通（保证地图定位不滞后）

- `BSP/bsp_imu.c`
  - `gyro_z` LPF -> yaw积分
  - pitch/roll 互补滤波通道
  - yaw 慢修正：仅在低角速 + 小倾角时启用微弱漂移衰减

- `Common/car_config.h`
  - 集中增加滤波系数与阈值宏，方便标定

- `Strategy/track_map.c`
  - 保留 Flash 地图存取：学习结束保存、上电优先加载

## 参数位置

所有滤波与漂移修正参数都在 `Common/car_config.h`：

- `LINE_RAW_LPF_ALPHA`
- `LINE_ERR_LPF_ALPHA`
- `ENC_SPD_LPF_ALPHA`
- `IMU_GYRO_LPF_ALPHA`
- `IMU_COMP_ALPHA`
- `YAW_DRIFT_FIX_*`

