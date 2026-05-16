#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * 分阶段测试功能开关：1U=启用，0U=关闭。
 * 建议顺序：先看传感器/OLED，再开电机，再开速度闭环，最后开地图学习/预测。
 */
#define CAR_ENABLE_LINE_SENSOR (1U)        // 循迹传感器采样开关
#define CAR_ENABLE_IMU (1U)                // MPU6050 姿态/角速度读取开关
#define CAR_ENABLE_ENCODER (1U)            // 编码器速度/里程读取开关
#define CAR_ENABLE_OLED (1U)               // OLED 调试显示开关
#define CAR_ENABLE_MOTOR_OUTPUT (1U)       // 编译进电机输出能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_SPEED_PID (1U)          // 编译进左右轮速度闭环能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_TRACK_MAP_LEARNING (1U) // 编译进第一圈地图学习能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_MAP_PREDICTION (1U)     // 编译进地图预测能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_LOST_PROTECTION (1U)    // 编译进丢线保护能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_PID_SCOPE (1U)          // 编译进 PIDScope 串口调参能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_HC05 (1U)               // HC-05 蓝牙串口驱动开关；当前使用 USART2 PA2/PA3
#define CAR_ENABLE_PID_SCOPE_OVER_HC05 (1U)// 编译进 PIDScope 复用 HC-05 的能力；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_REMOTE_CONTROL (1U)     // 上位机直接控制开关；通过 HC-05 文本命令实现
#define CAR_ENABLE_HC05_OLED_TEST (1U)     // 编译进 OLED 蓝牙收发测试页；默认是否启用由 runtime_config 控制
#define CAR_ENABLE_HC05_RX_ECHO_TEST (1U)  // 编译进蓝牙回显测试；默认是否启用由 runtime_config 控制

#define PID_SCOPE_DEVICE_ID (1U)           // 上位机设备号
#define PID_SCOPE_CH_LINE (0U)             // 通道0：循迹转向 PID
#define PID_SCOPE_CH_SPEED_LEFT (1U)       // 通道1：左轮速度 PID
#define PID_SCOPE_CH_SPEED_RIGHT (2U)      // 通道2：右轮速度 PID
#define PID_SCOPE_SEND_PERIOD_MS (500U)    // 9600 波特率下三通道遥测约 2Hz，给 CMD 回复和心跳留带宽

#define HC05_DEFAULT_BAUD (9600U)          // 常见 HC-05 数据模式默认波特率
#define HC05_AT_BAUD (38400U)              // 常见 HC-05 AT 模式默认波特率
#define HC05_RX_BUFFER_SIZE (128U)         // 蓝牙接收软件缓冲区
#define HC05_TX_BUFFER_SIZE (256U)         // 蓝牙发送软件缓冲区，后台非阻塞发送
#define HC05_TX_CHUNK_SIZE (32U)           // 每次 poll 最多交给底层串口的字节数
#define HC05_OLED_ACTIVITY_HOLD_MS (300U)  // OLED 上收发活动保持显示时间
#define HC05_TX_TIMEOUT_MS (100U)          // DMA 发送完成回调丢失时的恢复超时
#define REMOTE_CONTROL_TIMEOUT_MS (1000U)  // 上位机控制超时，超过后自动禁止电机

/* Control-loop timing and PWM limits. */
#define CAR_CTRL_DT_S (0.01f) // 控制周期
#define CAR_MAX_PWM (1000.0f) // 最大PWM
#define CAR_MIN_PWM (0.0f)    // 最小PWM

/* Line sensor weighted-position model: negative is left, positive is right. */
// 循迹误差值，用于位置PID
#define LINE_WEIGHT_L2 (-3.0f)
#define LINE_WEIGHT_L1 (-1.0f)
#define LINE_WEIGHT_R1 (1.0f)
#define LINE_WEIGHT_R2 (3.0f)

/* Strategy thresholds for line loss and straight-road prediction. */
#define LINE_LOST_TIMEOUT_MS (600U)      // 丢线超时时间
#define STRAIGHT_ERROR_TH (0.25f)        // 循线误差阈值
#define STRAIGHT_YAW_RATE_TH_DPS (12.0f) // 用来判断小车是否在发生剧烈的车身扭动。

/* Discrete target speed levels used by strategy and map events. */
// 速度档位，按你给的 60RPM 示例先保守设置；65mm 轮径时 60RPM 约等于 0.20m/s。
#define SPEED_LOW_MPS (0.10f)     // 低档：低速找线/初期调试
#define SPEED_MID_LOW_MPS (0.16f) // 中低档：弯道/地图风险段
#define SPEED_MID_MPS (0.20f)     // 中档：约等于 60RPM 的线速度
#define SPEED_HIGH_MPS (0.25f)    // 高档：初期测试不要太快，后续可逐步加大

/* Look-ahead distances reserved for future map-based prediction tuning. */
// 预测距离
#define PREDICT_DIST_LOW_M (0.08f)
#define PREDICT_DIST_MID_M (0.20f)
#define PREDICT_DIST_HIGH_M (0.35f)

/* Maximum number of learned map events kept in RAM for host-side export. */
#define MAP_MAX_EVENTS (64U)

/* Filter and yaw-drift correction coefficients; tune on the real vehicle. */
// 滤波器参数设置

// 循迹一阶低通滤波
#define LINE_RAW_LPF_ALPHA (0.35f)
#define LINE_ERR_LPF_ALPHA (0.45f)

// 编码器一阶低通滤波
#define ENC_SPD_LPF_ALPHA (0.30f)

// MPU一阶低通滤波
#define IMU_GYRO_LPF_ALPHA (0.20f)
#define IMU_COMP_ALPHA (0.96f)

#define YAW_DRIFT_FIX_RATE_TH_DPS (6.0f)        // 判断设备是否处于“静止或直线行驶”的角速度门限。
#define YAW_DRIFT_FIX_TILT_TH_DEG (8.0f)        // 判断设备是否处于“水平/平稳状态”的姿态角门限。
#define YAW_DRIFT_DECAY_WHEN_STRAIGHT (0.9995f) // 防止YAW漂移的

/* Hardware calibration values. */
#define LINE_SENSOR_ACTIVE_LEVEL (1U)    // 1U 代表 高电平代表扫线
#define ENCODER_HALL_PPR (11.0f)         // 电机霍尔一圈脉冲数
#define ENCODER_GEAR_RATIO (48.0f)       // 减速比，来自你给的 11x48x4
#define ENCODER_QUAD_FACTOR (4.0f)       // STM32 编码器四倍频
#define ENCODER_COUNTS_PER_REV (ENCODER_HALL_PPR * ENCODER_GEAR_RATIO * ENCODER_QUAD_FACTOR) // 输出轴每圈总脉冲数：2112
#define WHEEL_DIAMETER_M (0.065f)        // 车轮直径 m

#define MPU6050_I2C_ADDR (0x68U << 1)
#define MPU6050_GYRO_CALIB_SAMPLES (100U) // 校准采样次数，100U 代表 100 次
#define MPU6050_GYRO_CALIB_DELAY_MS (2U)  // 采样时间间隔
#define MPU6050_GYRO_Z_OFFSET_DPS (0.0f)  // Z 轴（偏航角 Yaw 轴）的预设手动补偿值。

#define OLED_I2C_ADDR (0x3CU << 1)

#endif
