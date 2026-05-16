# Application

整车应用层，负责固定周期控制流程编排。

- `app_car_init()` 初始化 PID、IMU、策略模块和地图状态。
- `app_car_run_10ms()` 执行 10 ms 控制循环。
- 本层只组织流程，不直接操作 STM32 外设寄存器。

当前 `Core/Src/stm32f1xx_it.c` 的 SysTick 中断只调用应用层 `app_car_on_1ms_tick()` 置位 10ms 控制任务；`Core/Src/main.c` 主循环调用 `app_car_run_scheduler()` 执行控制或后台切片。

- `app_car_run_10ms()`：快速控制任务，负责采样、策略、PID 和电机输出。
- `app_car_run_background()`：慢速后台任务，负责 OLED 和 PIDScope 等调试输出。
- `app_car_get_control_overrun()`：查看控制任务来不及处理时的过载计数。

调试阶段通过 `Common/car_config.h` 中的 `CAR_ENABLE_*` 开关逐步打开电机、速度闭环、地图学习和预测调速。需要保存的遥测/地图数据由上位机导出。

如启用 `CAR_ENABLE_PID_SCOPE`，本层会把循迹 PID 和速度 PID 绑定到 `Debug/PIDScope` 适配层，并在 10 ms 循环末尾轮询发送调参遥测。
