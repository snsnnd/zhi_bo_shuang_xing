# STM32 智能车循迹系统

本项目是基于 STM32F103C8T6 的智能车循迹控制工程，使用 STM32CubeMX 生成底层初始化代码，业务代码按应用层、板级抽象层、控制层和策略层拆分。

## 项目结构

- `Application/`：整车 10 ms 主控制循环，串联采样、学习、定位、策略、PID、电机输出和 OLED 调试。
- `BSP/`：板级抽象层，封装循迹、编码器、IMU、电机、OLED、HC-05 等真实硬件接口。
- `Common/`：全局类型、标定参数和运行期开关配置。
- `Control/`：循迹 PID 和左右轮速度 PID。
- `Strategy/`：地图学习和赛道策略决策，学习结果保存在 RAM 并由上位机导出。
- `Core/`：STM32CubeMX 生成的 HAL 初始化和中断入口，不建议直接修改生成区以外内容。
- `Drivers/`：ST HAL/CMSIS 厂商库。
- `MDK-ARM/`：Keil MDK 工程文件和启动文件。

## 核心流程

1. `app_car_init()` 初始化 PID、IMU、策略模块和 RAM 地图状态。
2. SysTick 每 1 ms 调用应用层 `app_car_on_1ms_tick()`，主循环通过 `app_car_run_scheduler()` 执行 10 ms 控制任务。
3. 第一圈未学习完成时，`track_map_learning_step()` 根据里程、yaw、循迹误差和外侧传感器自动生成路段事件。
4. 学习完成后地图事件留在 RAM，后续通过上位机通信导出保存，不再擦写 MCU Flash。
5. `track_strategy_step()` 根据当前路段、丢线状态和直道判断输出目标速度与转向限幅。
6. `line_pid_update()` 计算转向量，左右轮 `speed_pid_update()` 计算 PWM。

## 引脚定义

引脚配置来自 `aaa.ioc`、`Core/Src/gpio.c`、`Core/Src/tim.c` 和 `Core/Src/i2c.c`。

| 引脚 | 外设/功能 | 项目用途 |
| --- | --- | --- |
| PD0 | RCC_OSC_IN | HSE 外部晶振输入 |
| PD1 | RCC_OSC_OUT | HSE 外部晶振输出 |
| PA2 | USART2_TX | HC-05 蓝牙串口 TX，接 HC-05 RXD |
| PA3 | USART2_RX | HC-05 蓝牙串口 RX，接 HC-05 TXD |
| PA4 | GPIO_Input | 循迹 O1，最右侧 R2 |
| PA5 | GPIO_Input | 循迹 O2，右侧 R1 |
| PA6 | GPIO_Input | 循迹 O3，左侧 L1 |
| PA7 | GPIO_Input | 循迹 O4，最左侧 L2 |
| PA13 | SWDIO | SWD 下载/调试 |
| PA14 | SWCLK | SWD 下载/调试 |
| PA15 | TIM2_CH1 | 编码器 1 A 相，TIM2 部分重映射 |
| PB3 | TIM2_CH2 | 编码器 1 B 相，TIM2 部分重映射 |
| PB4 | TIM3_CH1 | 编码器 2 A 相，TIM3 部分重映射 |
| PB5 | TIM3_CH2 | 编码器 2 B 相，TIM3 部分重映射 |
| PB8 | TIM4_CH3 | TB6612 PWMB，B 右电机 PWM |
| PB9 | TIM4_CH4 | TB6612 PWMA，A 左电机 PWM |
| PB10 | I2C2_SCL | I2C 总线时钟，建议接 MPU6050/OLED |
| PB11 | I2C2_SDA | I2C 总线数据，建议接 MPU6050/OLED |
| PB12 | GPIO_Output | TB6612 BIN2，B 右电机方向 |
| PB13 | GPIO_Output | TB6612 BIN1，B 右电机方向 |
| PB14 | GPIO_Output | TB6612 AIN1，A 左电机方向 |
| PB15 | GPIO_Output | TB6612 AIN2，A 左电机方向 |

## BSP 硬件实现

- 循迹：`BSP/bsp_line.c` 直接读取 PA4-PA7；商家模块 O1/O2/O3/O4 从右到左排列，当前映射为 PA4=R2、PA5=R1、PA6=L1、PA7=L2。默认高电平为检测到黑线，可通过 `LINE_SENSOR_ACTIVE_LEVEL` 修改。
- 编码器：`BSP/bsp_encoder.c` 启动 TIM2/TIM3 编码器模式，通过计数差计算速度和里程；当前 TIM2=左 A 电机编码器 PA15/PB3，TIM3=右 B 电机编码器 PB4/PB5，均使用 TI12 四倍频模式。
- 电机：`BSP/bsp_motor.c` 启动 TIM4 CH3/CH4 PWM，PWM 频率配置为 20 kHz；左 A 电机使用 PB9/TIM4_CH4 + PB14/PB15，右 B 电机使用 PB8/TIM4_CH3 + PB13/PB12。
- IMU：`BSP/bsp_imu.c` 通过 I2C2 初始化和读取 MPU6050，积分补偿后的 gyro_z 得到 yaw。
- IMU 零漂：上电初始化时车辆需要静止，系统会采样 `MPU6050_GYRO_CALIB_SAMPLES` 次估算 gyro_z 零偏，再叠加手动偏置补偿 yaw 积分漂移。
- OLED：`BSP/bsp_oled.c` 通过 I2C2 驱动 SSD1306 兼容 128x64 屏，刷新限频到 10 Hz，避免阻塞控制周期。
- HC-05：`BSP/bsp_hc05.c` 提供蓝牙串口抽象驱动，不绑定具体 USART；配合 PIDScope 时可把调参协议通过 HC-05 发送到上位机。

## 关键参数

所有核心控制参数集中在 `Common/car_config.h`：

## 分阶段测试开关

`Common/car_config.h` 顶部提供编译能力开关，`1U` 表示把对应能力编译进固件，`0U` 表示完全裁剪。上电默认状态由 `Common/runtime_config.c` 控制：传感器、OLED、HC-05 和远控默认开启，电机输出、速度闭环、地图学习、地图预测、丢线保护和 PIDScope 默认关闭。

运行期可通过 HC-05 CMD 修改 feature，例如 `CFG LIST`、`CFG GET MOTOR_OUTPUT`、`CFG ARM MOTOR_OUTPUT`、`CFG SET MOTOR_OUTPUT 1`。安全开关 `MOTOR_OUTPUT`、`SPEED_PID`、`TRACK_MAP_LEARNING` 打开前必须先 arm，关闭不需要 arm。`CFG SET` 不能打开未编译进固件的能力。

建议测试顺序：

1. 只看数据：保持默认配置，确认循迹、IMU、编码器、OLED 数据正常。
2. 测电机方向：确认 `CAR_ENABLE_MOTOR_OUTPUT=1U` 已编译进固件，实车架空后执行 `CFG ARM MOTOR_OUTPUT`、`CFG SET MOTOR_OUTPUT 1`，低速观察左右轮方向。
3. 测速度闭环：确认 `CAR_ENABLE_SPEED_PID=1U` 已编译进固件，再执行 `CFG ARM SPEED_PID`、`CFG SET SPEED_PID 1`，确认编码器方向、轮径和脉冲数标定正确。
4. 测丢线保护：执行 `CFG SET LOST_PROTECTION 1`，确认传感器黑白逻辑稳定后再启用。
5. 测地图学习：执行 `CFG ARM TRACK_MAP_LEARNING`、`CFG SET TRACK_MAP_LEARNING 1`，观察学习到的路段是否合理。
6. 测上位机保存：连接 PIDScope/HC-05 上位机，使用保存按钮导出遥测 CSV。
7. 测预测调速：设置 `CAR_ENABLE_MAP_PREDICTION=1U`，让策略使用地图事件推荐速度。

主要开关：

- `CAR_ENABLE_LINE_SENSOR`：循迹采样。
- `CAR_ENABLE_IMU`：MPU6050 姿态/角速度读取。
- `CAR_ENABLE_ENCODER`：编码器速度/里程读取。
- `CAR_ENABLE_OLED`：OLED 调试显示。
- `CAR_ENABLE_MOTOR_OUTPUT`：真实电机输出能力；运行期默认关闭。
- `CAR_ENABLE_SPEED_PID`：左右轮速度闭环能力；运行期默认关闭。
- `CAR_ENABLE_TRACK_MAP_LEARNING`：第一圈地图学习能力；运行期默认关闭。
- `CAR_ENABLE_MAP_PREDICTION`：基于地图事件预测调速能力；运行期默认关闭。
- `CAR_ENABLE_LOST_PROTECTION`：丢线超时停车保护能力；运行期默认关闭。
- `CAR_ENABLE_PID_SCOPE`：PIDScope 串口调参适配能力；运行期默认关闭。
- `CAR_ENABLE_HC05`：HC-05 蓝牙串口驱动。
- `CAR_ENABLE_PID_SCOPE_OVER_HC05`：PIDScope 是否复用 HC-05 蓝牙通道。
- `CAR_ENABLE_REMOTE_CONTROL`：上位机直接控制车辆运行状态，依赖 HC-05 字节通道。

## HC-05 蓝牙模块

HC-05 与 MCU 通过 USART 连接，电脑端配对后会显示为普通串口，因此可直接给 PIDScope 上位机使用。

推荐连接：

- HC-05 `TXD` 接 MCU USART_RX。
- HC-05 `RXD` 接 MCU USART_TX，注意 HC-05 RXD 通常建议做 3.3V 电平或分压。
- HC-05 `VCC/GND` 接电源和地。
- 需要 AT 模式时再接 KEY/EN 引脚。

固件接入步骤：

1. 当前工程已配置 `USART2`：`PA2=TX`、`PA3=RX`，`DMA1_Channel7` 发送，`DMA1_Channel6` 接收。
2. 数据模式波特率由 `HC05_DEFAULT_BAUD` 控制，当前为 `9600`；如果用 AT 改过 HC-05 波特率，要同步修改该宏和 CubeMX USART2 波特率。
3. `BSP/bsp_hc05.c` 已适配 `huart2`：发送使用 `HAL_UART_Transmit_DMA()`，接收使用 1 字节 DMA 回调循环喂给 `bsp_hc05_rx_byte()`。
4. 设置 `CAR_ENABLE_HC05=1U`。
5. 如果 PIDScope 通过 HC-05 调参，再设置 `CAR_ENABLE_PID_SCOPE=1U` 和 `CAR_ENABLE_PID_SCOPE_OVER_HC05=1U`。

`BSP/bsp_hc05.c` 还提供 AT 辅助函数：`bsp_hc05_at_command()`、`bsp_hc05_at_set_name()`、`bsp_hc05_at_set_uart()`。这些函数只应在 HC-05 KEY/EN 进入 AT 模式时使用。

HC-05 发送接口是队列式非阻塞设计：`bsp_hc05_send()` 只把数据写入软件 TX 缓冲区，真正发送由后台 `bsp_hc05_poll()` 推进。底层 `bsp_hc05_port_write()` 建议使用 USART DMA 或中断发送；发送完成中断里调用 `bsp_hc05_tx_done()`。

## 上位机直接控制

`Control/remote_control.*` 负责上位机控制解析，BSP 只提供 HC-05 字节收发。协议是可读文本命令，兼容 `$PING` 这种老格式，也支持直接输入 `PING`，每条命令以换行结尾：

完整命令说明见 `Debug/CMD_MANUAL.md`。

- `HELP`：返回可用命令摘要。
- `PING`：连通性测试，返回 `$OK,PONG`。
- `START` / `STOP`：允许电机运行 / 立即禁止电机并进入强制停车状态。
- `AUTO`：退出模式和速度覆盖，恢复自动策略。
- `MOTOR 0` / `MOTOR 1`：禁止/允许电机输出。
- `MODE TRACKING`：覆盖小车模式，可选 `IDLE/CALIBRATION/LEARNING/TRACKING/PREDICTION/HAIRPIN/LOST/PROTECT`。
- `SPEED 0.15`：覆盖目标速度，单位 m/s。
- `LIMIT 0.25`：设置运行时速度上限，最大不超过 `SPEED_HIGH_MPS`。
- `CFG ?`：查询关键运行态配置摘要，例如 `MOTOR_OUT`、`SPEED_PID`、`TIMEOUT`、`LIMIT`。
- `CFG LIST`：逐行列出所有运行期 feature 状态。
- `CFG GET/ARM/SET <feature>`：查询、预确认或修改运行期 feature。
- `CFG TIMEOUT 3000`：把远控超时改为 3000ms。
- `PID LINE 120 0 25`：设置循迹 PID 的 `kp/ki/kd` 并清积分。
- `PID LEFT 500 50 0`、`PID RIGHT ...`、`PID SPEED ...`：设置左轮、右轮或双轮速度 PID；速度 PID 需要 `CAR_ENABLE_SPEED_PID=1U` 才会应用。
- `STATUS` / `STATUS?`：查询当前电机、停车、速度限制和模式状态。

安全约束：

- 未编译 `CAR_ENABLE_MOTOR_OUTPUT=1U` 或运行期未打开 `MOTOR_OUTPUT` 时，上位机无法真正启动电机。
- `CAR_ENABLE_REMOTE_CONTROL=1U` 且超时超过 `REMOTE_CONTROL_TIMEOUT_MS` 未收到命令时，会自动禁止电机并强制停车。
- 如果要让上位机控制电机，需要编译进 `CAR_ENABLE_HC05=1U`、`CAR_ENABLE_REMOTE_CONTROL=1U`、`CAR_ENABLE_MOTOR_OUTPUT=1U`，并在运行期打开 `MOTOR_OUTPUT`。
- 命令只修改 RAM 运行态参数，掉电不会保存；确认参数后再写回 `car_config.h` 或 PID 初始化值。

## 算力与非阻塞要求

当前芯片 STM32F103C8T6 是 Cortex-M3 72MHz、20KB SRAM、64KB Flash 级别。按当前代码规模和你之前的 Keil 输出 `Code≈16KB, ZI≈5KB`，基础循迹、编码器、双 PID、简单策略、HC-05 和 PIDScope 协议解析的算力是够的。真正容易超时的不是 PID 计算，而是 I2C OLED、串口阻塞发送和过高频率的调参遥测。

因此项目采用以下约束：

- 10ms 控制任务只做采样、策略、PID、电机输出和缓存调试数据。
- OLED、PIDScope、HC-05 发送放在 `app_car_run_background()`。
- HC-05 使用 TX 环形缓冲，后台 `bsp_hc05_poll()` 非阻塞推进。
- PIDScope 自带发送环形缓冲，适合后台轮询。
- MCU 不再擦写内部 Flash 保存地图；需要保留的数据由上位机导出，避免 Flash 擦写阻塞控制。
- 若启用 USART，优先用 DMA 或中断发送；不要在 10ms 控制路径里调用长时间阻塞的 `HAL_UART_Transmit()`。

## PIDScope 调参工具

你提供的 `https://github.com/snsnnd/PID_Parameter_Tuning.git` 是一个 PySide6 串口上位机，使用二进制协议传输 PID 遥测和参数。协议帧格式为：

- 帧头：`0xA5 0x5A`
- 版本：`0x01`
- 类型：`0x01` 遥测，`0x02` 写参数，`0x03` 参数回报
- 设备号：当前工程默认 `PID_SCOPE_DEVICE_ID=1`
- 通道号：`0` 循迹转向 PID，`1` 左轮速度 PID，`2` 右轮速度 PID
- 长度：小端 `uint16_t`
- 载荷：遥测为 `timestamp,target,feedback,error,output,kp,ki,kd,p/i/d,extra1,extra2,status`
- CRC：Modbus CRC16，小端

本工程已新增 `Debug/PIDScope/` 独立适配层，并已接到 HC-05/USART2 字节通道。二进制 PIDScope 波形调参和 `Control/remote_control.*` 的文本 CMD 命令共用同一个蓝牙串口。

1. 在 `PID_Parameter_Tuning` 目录执行 `pip install -r requirements.txt`，再运行 `python app.py`。
2. 当前固件使用 `USART2 PA2/PA3` 连接 HC-05，波特率为 `HC05_DEFAULT_BAUD`。
3. `Debug/PIDScope/pid_debug.c` 和 `Debug/PIDScope/pid_scope_adapter.c` 已加入 Keil 工程。
4. `pid_scope_port_write()` 默认走 `bsp_hc05_send()`，由 USART2 DMA 非阻塞发送。
5. HC-05 RX 回调会同时把字节喂给 PIDScope 二进制解析器和 CMD 文本解析器。
6. 默认已编译进 `CAR_ENABLE_PID_SCOPE=1U`、`CAR_ENABLE_HC05=1U`、`CAR_ENABLE_PID_SCOPE_OVER_HC05=1U`，但运行期 `PID_SCOPE` 和 `PID_SCOPE_OVER_HC05` 默认关闭，避免上电就向蓝牙发送二进制流。
7. 如果要调左右轮速度 PID，还要运行期打开 `SPEED_PID`。
8. 上位机选择 HC-05 对应串口和波特率连接，点击“开启波形”后观察 `target/feedback/error/output` 波形，在 Advanced Tuning 页写入 PID 参数。

注意：PIDScope 二进制波形、CMD 文本回复和悬空测试心跳共用同一个 HC-05 串口。当前默认使用 9600 波特率，`PID_SCOPE_SEND_PERIOD_MS=150U`，且只发送上位机当前选择的通道，用于给 CMD 回复保留带宽；开启波形后不要频繁执行 `CFG LIST`。

注意：写入参数是 RAM 内实时生效，掉电不会保存。确认参数后再手工写回 `Application/app_car.c` 的 PID 初始化值。

- `CAR_CTRL_DT_S`：控制周期，当前为 10 ms。
- `CAR_MAX_PWM`、`CAR_MIN_PWM`：PWM 输出限幅。
- `LINE_WEIGHT_*`：四路循迹传感器权重，决定误差方向和大小。
- `SPEED_*_MPS`：策略层速度档位。
- `MAP_MAX_EVENTS`：RAM 中最多保留的地图事件数量。
- `LINE_RAW_LPF_ALPHA`、`LINE_ERR_LPF_ALPHA`：循迹值和循迹误差低通滤波系数。
- `ENC_SPD_LPF_ALPHA`：编码器速度低通滤波系数。
- `IMU_GYRO_LPF_ALPHA`、`IMU_COMP_ALPHA`：IMU gyro/yaw 和 pitch/roll 滤波系数。
- `YAW_DRIFT_FIX_*`：直行近水平时的 yaw 慢漂移修正阈值。
- `ENCODER_HALL_PPR`、`ENCODER_GEAR_RATIO`、`ENCODER_QUAD_FACTOR`：编码器物理参数，当前按 `11 x 48 x 4 = 2112` 计数/输出轴一圈。
- `ENCODER_COUNTS_PER_REV`、`WHEEL_DIAMETER_M`：编码器总脉冲数和车轮直径，用于速度/里程换算。
- `SPEED_*_MPS`：速度档位，当前按 65mm 轮径、约 60RPM 对应 0.20m/s 做保守初始值。
- `MPU6050_GYRO_Z_OFFSET_DPS`：陀螺仪 Z 轴零偏。
- `MPU6050_GYRO_CALIB_SAMPLES`、`MPU6050_GYRO_CALIB_DELAY_MS`：上电 gyro_z 自动零漂校准采样参数。

## 中断策略

当前工程使用 SysTick 作为 1ms 调度源，中断入口只调用应用层 `app_car_on_1ms_tick()` 置位 10ms 控制任务，不在中断里执行 PID/OLED/I2C。

- 必须保留 SysTick 中断：HAL 依赖 SysTick 维护 `HAL_GetTick()`，当前 10 ms 调度也依赖它。
- 暂不需要编码器中断：TIM2/TIM3 硬件计数器可在 10 ms 周期内读取差值，CPU 开销小且更稳定。
- 暂不需要循迹 GPIO 中断：循迹是连续采样信号，周期轮询比边沿中断更适合做滤波和 PID。
- 暂不建议在中断里跑 `app_car_run_10ms()`：其中包含 I2C OLED/IMU、PID 和策略逻辑，放进 ISR 会造成阻塞和优先级问题。
- 主循环调用 `app_car_run_scheduler()`：有控制任务到期就运行 `app_car_run_10ms()`，否则运行 `app_car_run_background()`。
- 如果上一个 10ms 控制周期还没处理完，应用层只记录 overrun，不补跑过期 PID；可通过 `app_car_get_control_overrun()` 查看过载次数。
- 后续如需改成 TIMx，只要在 TIM 中断里调用 `app_car_on_1ms_tick()` 或等价的应用层 tick 接口即可。

## CubeMX 生成文件说明

以下内容由 STM32CubeMX 或 ST 库生成/提供，更新 `.ioc` 后可能被重新生成覆盖：

- `aaa.ioc`
- `.mxproject`
- `Core/`
- `Drivers/`
- `MDK-ARM/` 中的工程和启动文件

业务逻辑建议放在 `Application/`、`BSP/`、`Common/`、`Control/`、`Strategy/`。如必须修改 CubeMX 生成文件，应优先写在 `USER CODE BEGIN/END` 区域内。

## 实车注意事项

- 地图和调参数据不写 MCU 内部 Flash，需要保存时使用上位机导出文件。
- `ENCODER_COUNTS_PER_REV` 当前来自 `11 x 48 x 4 = 2112`；如果更换电机减速比，要同步修改 `ENCODER_GEAR_RATIO`。
- `WHEEL_DIAMETER_M` 需要按实车轮径测量值修正。
- 上电零漂校准期间车辆必须静止，否则 yaw 会带入错误偏置。
- 如果电机方向反了，优先调整电机接线或 `bsp_motor.c` 中 PB12-PB15 的方向映射。
- 如果循迹黑白逻辑反了，修改 `LINE_SENSOR_ACTIVE_LEVEL`。
