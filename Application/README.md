# Application

整车应用层，负责固定周期控制流程编排。

- `app_car_init()` 初始化 PID、IMU、策略模块和地图状态。
- `app_car_run_10ms()` 执行 10 ms 控制循环。
- 本层只组织流程，不直接操作 STM32 外设寄存器。
