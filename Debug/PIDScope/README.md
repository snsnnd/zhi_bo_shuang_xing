# PIDScope Adapter

适配 `https://github.com/snsnnd/PID_Parameter_Tuning.git` 的固件端协议。

- `pid_debug.*`：上位机二进制协议、CRC、发送环形缓冲和 PARAM_SET 解析。
- `pid_scope_adapter.*`：把本工程的 `line_pid_t`、`speed_pid_t` 映射到 PIDScope 多通道模型。

当前工程已配置 USART2 PA2/PA3 给 HC-05，PIDScope 默认复用 HC-05 字节通道。需要使用时：

1. 保持 `CAR_ENABLE_PID_SCOPE=1U`、`CAR_ENABLE_HC05=1U`、`CAR_ENABLE_PID_SCOPE_OVER_HC05=1U`。
2. 保持 `Debug/PIDScope/pid_debug.c` 和 `Debug/PIDScope/pid_scope_adapter.c` 在 Keil 工程中。
3. `pid_scope_port_write()` 默认调用 `bsp_hc05_send()`，最终由 USART2 DMA 非阻塞发送。
4. HC-05 接收回调会同时喂给 `pid_scope_rx_byte(byte)` 和 `remote_control_rx_byte(byte)`。
5. 如果要调左右轮速度 PID，再设置 `CAR_ENABLE_SPEED_PID=1U`。
