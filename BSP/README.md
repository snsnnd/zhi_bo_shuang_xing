# BSP

板级抽象层，隔离业务逻辑和具体硬件驱动。

- `bsp_line.*`：四路循迹采样、二值化和误差计算。
- `bsp_encoder.*`：左右轮速度和里程状态。
- `bsp_imu.*`：yaw、gyro_z、pitch、roll 状态融合。
- `bsp_motor.*`：左右轮 PWM 和方向命令接口。
- `bsp_oled.*`：运行状态调试显示。
- `bsp_flash.*`：地图 Flash 持久化接口。

## 当前硬件映射

- 循迹：PA4/PA5/PA6/PA7，对应 L2/L1/R1/R2，默认高电平表示检测到线。
- 编码器：TIM2 读取左轮，TIM3 读取右轮，速度由计数差和 `ENCODER_COUNTS_PER_REV` 换算。
- 电机：TIM4 CH3/CH4 输出左右轮 PWM，PB12/PB13 控制左轮方向，PB14/PB15 控制右轮方向。
- IMU：I2C2 读取 MPU6050，地址由 `MPU6050_I2C_ADDR` 配置，上电静止采样补偿 gyro_z 零漂。
- OLED：I2C2 驱动 SSD1306 兼容 128x64 OLED，地址由 `OLED_I2C_ADDR` 配置，刷新限频到 10 Hz。
- Flash：使用 STM32F1 内部 Flash 最后 2KB 区域保存地图，地址由 `FLASH_MAP_ADDR` 配置。

实车调试时优先校准 `Common/car_config.h` 中的循迹电平、编码器线数、轮径、IMU 自动零漂采样参数/手动零偏和 Flash 地址。
