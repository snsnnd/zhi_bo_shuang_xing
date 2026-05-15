# BSP

板级抽象层，隔离业务逻辑和具体硬件驱动。

- `bsp_line.*`：四路循迹采样、二值化和误差计算。
- `bsp_encoder.*`：左右轮速度和里程状态。
- `bsp_imu.*`：yaw、gyro_z、pitch、roll 状态融合。
- `bsp_motor.*`：左右轮 PWM 和方向命令接口。
- `bsp_oled.*`：运行状态调试显示。
- `bsp_flash.*`：地图 Flash 持久化接口。

当前部分文件是 mock/抽象实现，移植到实车时在本目录替换底层读写逻辑即可。
