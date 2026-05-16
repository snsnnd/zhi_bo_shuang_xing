# Control

闭环控制算法目录。

- `line_pid.*`：将循迹误差转换为差速转向修正量。
- `speed_pid.*`：将左右轮目标速度和反馈速度转换为 PWM 输出。
- `remote_control.*`：解析上位机/蓝牙文本命令，输出运行时控制状态。

PID 初始化参数目前在 `Application/app_car.c` 中设置，实车调试时可以通过上位机命令 `PID LINE/LEFT/RIGHT/SPEED kp ki kd` 在线调整，命令生效后会清理对应 PID 历史状态。

PID 结构体会记录最近一次 `target/feedback/error/output/p_term/i_term/d_term`，供 PIDScope 调参工具显示波形和分析响应。

远程控制协议使用换行结尾的文本命令，便于蓝牙串口助手和 PID 上位机同时调试；兼容 `$PING` 和 `PING` 两种输入。常用命令：`HELP`、`START`、`STOP`、`AUTO`、`MOTOR 0/1`、`MODE TRACKING`、`SPEED 0.15`、`LIMIT 0.25`、`CFG ?`、`CFG LIST`、`CFG GET/ARM/SET <feature>`、`CFG TIMEOUT 3000`、`PID LINE 120 0 25`、`STATUS`。完整说明见 `Debug/CMD_MANUAL.md`。
