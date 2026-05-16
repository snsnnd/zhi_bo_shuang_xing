# STM32 智能车循迹系统

本项目是基于 STM32F103C8T6 的智能车循迹控制工程，使用 STM32CubeMX 生成底层初始化代码，业务代码按应用层、板级抽象层、控制层和策略层拆分。

## 项目结构

- `Application/`：整车 10 ms 主控制循环，串联采样、学习、定位、策略、PID、电机输出和 OLED 调试。
- `BSP/`：板级抽象层，封装循迹、编码器、IMU、电机、OLED、Flash 等真实硬件接口。
- `Common/`：全局类型和标定参数。
- `Control/`：循迹 PID 和左右轮速度 PID。
- `Strategy/`：地图学习、Flash 持久化和赛道策略决策。
- `Core/`：STM32CubeMX 生成的 HAL 初始化和中断入口，不建议直接修改生成区以外内容。
- `Drivers/`：ST HAL/CMSIS 厂商库。
- `MDK-ARM/`：Keil MDK 工程文件和启动文件。

## 核心流程

1. `app_car_init()` 初始化 PID、IMU、策略模块，并优先从 Flash 读取已学习地图。
2. `main.c` 使用 `HAL_GetTick()` 对 `app_car_run_10ms()` 做 10 ms 节拍调度。
3. 第一圈未学习完成时，`track_map_learning_step()` 根据里程、yaw、循迹误差和外侧传感器自动生成路段事件。
4. 学习完成后通过 `track_map_save_to_flash()` 保存地图，上电后可直接加载进入循迹。
5. `track_strategy_step()` 根据当前路段、丢线状态和直道判断输出目标速度与转向限幅。
6. `line_pid_update()` 计算转向量，左右轮 `speed_pid_update()` 计算 PWM。

## 引脚定义

引脚配置来自 `aaa.ioc`、`Core/Src/gpio.c`、`Core/Src/tim.c` 和 `Core/Src/i2c.c`。

| 引脚 | 外设/功能 | 项目用途 |
| --- | --- | --- |
| PD0 | RCC_OSC_IN | HSE 外部晶振输入 |
| PD1 | RCC_OSC_OUT | HSE 外部晶振输出 |
| PA4 | GPIO_Input | 循迹输入，建议接 L2 |
| PA5 | GPIO_Input | 循迹输入，建议接 L1 |
| PA6 | GPIO_Input | 循迹输入，建议接 R1 |
| PA7 | GPIO_Input | 循迹输入，建议接 R2 |
| PA13 | SWDIO | SWD 下载/调试 |
| PA14 | SWCLK | SWD 下载/调试 |
| PA15 | TIM2_CH1 | 编码器 1 A 相，TIM2 部分重映射 |
| PB3 | TIM2_CH2 | 编码器 1 B 相，TIM2 部分重映射 |
| PB4 | TIM3_CH1 | 编码器 2 A 相，TIM3 部分重映射 |
| PB5 | TIM3_CH2 | 编码器 2 B 相，TIM3 部分重映射 |
| PB8 | TIM4_CH3 | 电机 PWM 输出 1 |
| PB9 | TIM4_CH4 | 电机 PWM 输出 2 |
| PB10 | I2C2_SCL | I2C 总线时钟，建议接 MPU6050/OLED |
| PB11 | I2C2_SDA | I2C 总线数据，建议接 MPU6050/OLED |
| PB12 | GPIO_Output | 电机方向/控制输出 1 |
| PB13 | GPIO_Output | 电机方向/控制输出 2 |
| PB14 | GPIO_Output | 电机方向/控制输出 3 |
| PB15 | GPIO_Output | 电机方向/控制输出 4 |

## BSP 硬件实现

- 循迹：`BSP/bsp_line.c` 直接读取 PA4-PA7，默认高电平为检测到线，可通过 `LINE_SENSOR_ACTIVE_LEVEL` 修改。
- 编码器：`BSP/bsp_encoder.c` 启动 TIM2/TIM3 编码器模式，通过计数差计算速度和里程。
- 电机：`BSP/bsp_motor.c` 启动 TIM4 CH3/CH4 PWM，并用 PB12-PB15 控制左右轮方向。
- IMU：`BSP/bsp_imu.c` 通过 I2C2 初始化和读取 MPU6050，积分补偿后的 gyro_z 得到 yaw。
- IMU 零漂：上电初始化时车辆需要静止，系统会采样 `MPU6050_GYRO_CALIB_SAMPLES` 次估算 gyro_z 零偏，再叠加手动偏置补偿 yaw 积分漂移。
- OLED：`BSP/bsp_oled.c` 通过 I2C2 驱动 SSD1306 兼容 128x64 屏，刷新限频到 10 Hz，避免阻塞控制周期。
- Flash：`BSP/bsp_flash.c` 使用 STM32F1 内部 Flash 半字编程，在 `FLASH_MAP_ADDR` 起始的 2KB 区域保存地图。

## 关键参数

所有核心控制参数集中在 `Common/car_config.h`：

## 分阶段测试开关

`Common/car_config.h` 顶部提供软件功能开关，`1U` 表示启用，`0U` 表示关闭。当前默认偏安全：传感器和 OLED 开启，电机输出、速度闭环、地图学习、Flash 地图、地图预测和丢线保护关闭。

建议测试顺序：

1. 只看数据：保持默认配置，确认循迹、IMU、编码器、OLED 数据正常。
2. 测电机方向：设置 `CAR_ENABLE_MOTOR_OUTPUT=1U`，仍保持 `CAR_ENABLE_SPEED_PID=0U`，低速观察左右轮方向。
3. 测速度闭环：设置 `CAR_ENABLE_SPEED_PID=1U`，确认编码器方向、轮径和脉冲数标定正确。
4. 测丢线保护：设置 `CAR_ENABLE_LOST_PROTECTION=1U`，确认传感器黑白逻辑稳定后再启用。
5. 测地图学习：设置 `CAR_ENABLE_TRACK_MAP_LEARNING=1U`，先不要开 Flash，观察学习到的路段是否合理。
6. 测地图持久化：设置 `CAR_ENABLE_TRACK_MAP_FLASH=1U`，确认 Flash 地址不会和程序区冲突后再保存地图。
7. 测预测调速：设置 `CAR_ENABLE_MAP_PREDICTION=1U`，让策略使用地图事件推荐速度。

主要开关：

- `CAR_ENABLE_LINE_SENSOR`：循迹采样。
- `CAR_ENABLE_IMU`：MPU6050 姿态/角速度读取。
- `CAR_ENABLE_ENCODER`：编码器速度/里程读取。
- `CAR_ENABLE_OLED`：OLED 调试显示。
- `CAR_ENABLE_MOTOR_OUTPUT`：真实电机输出，首次上电建议保持关闭。
- `CAR_ENABLE_SPEED_PID`：左右轮速度闭环。
- `CAR_ENABLE_TRACK_MAP_LEARNING`：第一圈地图学习。
- `CAR_ENABLE_TRACK_MAP_FLASH`：地图 Flash 保存/加载。
- `CAR_ENABLE_MAP_PREDICTION`：基于地图事件预测调速。
- `CAR_ENABLE_LOST_PROTECTION`：丢线超时停车保护。

- `CAR_CTRL_DT_S`：控制周期，当前为 10 ms。
- `CAR_MAX_PWM`、`CAR_MIN_PWM`：PWM 输出限幅。
- `LINE_WEIGHT_*`：四路循迹传感器权重，决定误差方向和大小。
- `SPEED_*_MPS`：策略层速度档位。
- `MAP_MAX_EVENTS`：Flash 中最多保存的地图事件数量。
- `LINE_RAW_LPF_ALPHA`、`LINE_ERR_LPF_ALPHA`：循迹值和循迹误差低通滤波系数。
- `ENC_SPD_LPF_ALPHA`：编码器速度低通滤波系数。
- `IMU_GYRO_LPF_ALPHA`、`IMU_COMP_ALPHA`：IMU gyro/yaw 和 pitch/roll 滤波系数。
- `YAW_DRIFT_FIX_*`：直行近水平时的 yaw 慢漂移修正阈值。
- `ENCODER_COUNTS_PER_REV`、`WHEEL_DIAMETER_M`：编码器和车轮标定参数。
- `MPU6050_GYRO_Z_OFFSET_DPS`：陀螺仪 Z 轴零偏。
- `MPU6050_GYRO_CALIB_SAMPLES`、`MPU6050_GYRO_CALIB_DELAY_MS`：上电 gyro_z 自动零漂校准采样参数。
- `FLASH_MAP_ADDR`、`FLASH_MAP_SIZE_BYTES`：地图持久化 Flash 区域。

## 中断策略

当前工程建议先保持主循环 10 ms 调度，不把整车控制逻辑放进中断。

- 必须保留 SysTick 中断：HAL 依赖 SysTick 维护 `HAL_GetTick()`，当前 10 ms 调度也依赖它。
- 暂不需要编码器中断：TIM2/TIM3 硬件计数器可在 10 ms 周期内读取差值，CPU 开销小且更稳定。
- 暂不需要循迹 GPIO 中断：循迹是连续采样信号，周期轮询比边沿中断更适合做滤波和 PID。
- 暂不建议在中断里跑 `app_car_run_10ms()`：其中包含 I2C OLED/IMU、Flash、PID 和策略逻辑，放进 ISR 会造成阻塞和优先级问题。
- 后续如需更稳定周期，可新增一个基础定时器中断只置位 `g_ctrl_10ms_flag`，主循环看到标志后再执行 `app_car_run_10ms()`。

## CubeMX 生成文件说明

以下内容由 STM32CubeMX 或 ST 库生成/提供，更新 `.ioc` 后可能被重新生成覆盖：

- `aaa.ioc`
- `.mxproject`
- `Core/`
- `Drivers/`
- `MDK-ARM/` 中的工程和启动文件

业务逻辑建议放在 `Application/`、`BSP/`、`Common/`、`Control/`、`Strategy/`。如必须修改 CubeMX 生成文件，应优先写在 `USER CODE BEGIN/END` 区域内。

## 实车注意事项

- `FLASH_MAP_ADDR=0x0800F800` 会占用 STM32F103C8 Flash 末尾 2KB，固件链接大小必须避开该区域。
- `ENCODER_COUNTS_PER_REV` 需要按编码器线数、倍频方式和减速比重新标定。
- `WHEEL_DIAMETER_M` 需要按实车轮径测量值修正。
- 上电零漂校准期间车辆必须静止，否则 yaw 会带入错误偏置。
- 如果电机方向反了，优先调整电机接线或 `bsp_motor.c` 中 PB12-PB15 的方向映射。
- 如果循迹黑白逻辑反了，修改 `LINE_SENSOR_ACTIVE_LEVEL`。
