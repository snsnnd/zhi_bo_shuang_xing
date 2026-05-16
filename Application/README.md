# Application

整车应用层，负责固定周期控制流程编排。

- `app_car_init()` 初始化 PID、IMU、策略模块和地图状态。
- `app_car_run_10ms()` 执行 10 ms 控制循环。
- 本层只组织流程，不直接操作 STM32 外设寄存器。

当前 `Core/Src/main.c` 在 `USER CODE` 区域使用 `HAL_GetTick()` 做 10 ms 调度，避免主循环无延时地反复执行控制周期。

调试阶段通过 `Common/car_config.h` 中的 `CAR_ENABLE_*` 开关逐步打开电机、速度闭环、地图学习、Flash 保存和预测调速。
