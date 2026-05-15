# STM32 智能车循迹系统（完整骨架+学习+预测+丢线恢复+OLED三页面）

## 已实现内容

- 第一圈自动学习事件识别：基于误差符号翻转、yaw角变化、外侧传感器触发识别直线/S弯/U弯/急弯并自动打点。
- 丢线恢复状态机：进入 LOST 后低速搜索；超过超时阈值自动进入 PROTECT 停车保护。
- OLED 三页面真实字段渲染：
  - 页面1 传感器页（四路原始值、二值状态、误差、模式）
  - 页面2 运动页（左右PWM、左右速度、累计距离、yaw）
  - 页面3 地图页（当前路段ID/类型、到下一个事件距离、当前限速）
- IMU 一阶低通滤波（gyro z）+ yaw 积分。

## 核心文件

- `Application/app_car.c`：10ms闭环调度、学习逻辑、状态机执行、恢复策略、OLED数据装配。
- `Strategy/track_map.c`：事件表容器与学习识别逻辑。
- `Strategy/track_strategy.c`：策略输出（模式、目标速度、转向限幅）与丢线超时保护。
- `BSP/bsp_oled.c`：三页面字段渲染示例（目前使用 `printf`，可替换为SSD1306绘制函数）。
- `BSP/bsp_imu.c`：低通滤波与航向积分。

## 集成说明（CubeMX）

1. 保持 `app_car_init()` 在系统启动后调用。  
2. 使用 10ms 周期调用 `app_car_run_10ms()`。  
3. 将 BSP 中 mock 接口替换为 HAL 真实读写：
   - line -> ADC/DMA
   - imu -> I2C读取MPU6050
   - encoder -> TIM Encoder
   - motor -> PWM + GPIO(TB6612)
   - oled -> SSD1306绘制API

