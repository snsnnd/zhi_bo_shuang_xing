# BSP

板级抽象层，隔离业务逻辑和具体硬件驱动。

- `bsp_line.*`：四路循迹采样、二值化和误差计算。
- `bsp_encoder.*`：左右轮速度和里程状态。
- `bsp_imu.*`：yaw、gyro_z、pitch、roll 状态融合。
- `bsp_motor.*`：左右轮 PWM 和方向命令接口。
- `bsp_oled.*`：运行状态调试显示。
- `bsp_hc05.*`：HC-05 蓝牙串口抽象驱动。

## 当前硬件映射

- 循迹：商家模块 O1/O2/O3/O4 从右到左排列，实际接线为 O1-PA4=R2、O2-PA5=R1、O3-PA6=L1、O4-PA7=L2，默认高电平表示检测到黑线。
- 编码器：TIM2 读取左 A 电机编码器 PA15/PB3，TIM3 读取右 B 电机编码器 PB4/PB5，均使用 TI12 编码器模式；速度由计数差和 `ENCODER_COUNTS_PER_REV` 换算。
- 电机：TIM4 输出 20 kHz PWM；左 A 电机使用 PB9/TIM4_CH4 + PB14/PB15，右 B 电机使用 PB8/TIM4_CH3 + PB13/PB12，对齐商家 TB6612 参考接线。
- IMU：I2C2 读取 MPU6050，地址由 `MPU6050_I2C_ADDR` 配置，上电静止采样补偿 gyro_z 零漂。
- OLED：I2C2 驱动 SSD1306 兼容 128x64 OLED，地址由 `OLED_I2C_ADDR` 配置，刷新限频到 10 Hz。
- HC-05：当前适配 USART2，PA2 接 HC-05 RXD，PA3 接 HC-05 TXD；TX 使用 DMA 非阻塞发送，RX 使用 1 字节 DMA 回调循环接收。

实车调试时优先校准 `Common/car_config.h` 中的循迹电平、编码器线数、轮径、IMU 自动零漂采样参数和手动零偏。
