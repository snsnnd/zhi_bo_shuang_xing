# STM32 智能车循迹系统

本项目是基于 STM32F103C8T6 的智能车循迹控制工程，使用 STM32CubeMX 生成底层初始化代码，业务代码按应用层、板级抽象层、控制层和策略层拆分。

## 项目结构

- `Application/`：整车 10 ms 主控制循环，串联采样、学习、定位、策略、PID、电机输出和 OLED 调试。
- `BSP/`：板级抽象层，封装循迹、编码器、IMU、电机、OLED、Flash 等硬件接口，目前部分实现是便于联调的 mock。
- `Common/`：全局类型和标定参数。
- `Control/`：循迹 PID 和左右轮速度 PID。
- `Strategy/`：地图学习、Flash 持久化和赛道策略决策。
- `Core/`：STM32CubeMX 生成的 HAL 初始化和中断入口，不建议直接修改生成区以外内容。
- `Drivers/`：ST HAL/CMSIS 厂商库。
- `MDK-ARM/`：Keil MDK 工程文件和启动文件。

## 核心流程

1. `app_car_init()` 初始化 PID、IMU、策略模块，并优先从 Flash 读取已学习地图。
2. `app_car_run_10ms()` 每 10 ms 执行一次控制周期。
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

## 关键参数

所有核心控制参数集中在 `Common/car_config.h`：

- `CAR_CTRL_DT_S`：控制周期，当前为 10 ms。
- `CAR_MAX_PWM`、`CAR_MIN_PWM`：PWM 输出限幅。
- `LINE_WEIGHT_*`：四路循迹传感器权重，决定误差方向和大小。
- `SPEED_*_MPS`：策略层速度档位。
- `MAP_MAX_EVENTS`：Flash 中最多保存的地图事件数量。
- `LINE_RAW_LPF_ALPHA`、`LINE_ERR_LPF_ALPHA`：循迹值和循迹误差低通滤波系数。
- `ENC_SPD_LPF_ALPHA`：编码器速度低通滤波系数。
- `IMU_GYRO_LPF_ALPHA`、`IMU_COMP_ALPHA`：IMU gyro/yaw 和 pitch/roll 滤波系数。
- `YAW_DRIFT_FIX_*`：直行近水平时的 yaw 慢漂移修正阈值。

## CubeMX 生成文件说明

以下内容由 STM32CubeMX 或 ST 库生成/提供，更新 `.ioc` 后可能被重新生成覆盖：

- `aaa.ioc`
- `.mxproject`
- `Core/`
- `Drivers/`
- `MDK-ARM/` 中的工程和启动文件

业务逻辑建议放在 `Application/`、`BSP/`、`Common/`、`Control/`、`Strategy/`。如必须修改 CubeMX 生成文件，应优先写在 `USER CODE BEGIN/END` 区域内。

## 移植到实车注意事项

- `BSP/bsp_flash.c` 当前是 RAM mock，需要替换为 STM32 Flash 擦写实现。
- `BSP/bsp_oled.c` 当前用 `printf` 模拟显示，需要替换为 SSD1306 或实际 OLED 驱动。
- `BSP/bsp_motor.c` 当前只缓存命令，需要接入 TIM4 PWM 和 PB12-PB15 方向控制。
- `BSP/bsp_line.c` 当前由 `bsp_line_set_raw()` 注入归一化数据，实车需要接入 GPIO/ADC 采样。
- `BSP/bsp_imu.c` 当前接口接收已换算的 gyro/acc 数据，实车需要在 I2C2 上读取 MPU6050 等 IMU。
