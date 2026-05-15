# Control

闭环控制算法目录。

- `line_pid.*`：将循迹误差转换为差速转向修正量。
- `speed_pid.*`：将左右轮目标速度和反馈速度转换为 PWM 输出。

PID 初始化参数目前在 `Application/app_car.c` 中设置，实车调试时建议结合遥测逐步调整。
