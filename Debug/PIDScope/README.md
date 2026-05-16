# PIDScope Adapter

适配 `https://github.com/snsnnd/PID_Parameter_Tuning.git` 的固件端协议。

- `pid_debug.*`：上位机二进制协议、CRC、发送环形缓冲和 PARAM_SET 解析。
- `pid_scope_adapter.*`：把本工程的 `line_pid_t`、`speed_pid_t` 映射到 PIDScope 多通道模型。

当前工程未配置 USART，默认 `CAR_ENABLE_PID_SCOPE=0`。需要使用时：

1. 在 CubeMX 打开一个 USART，例如 USART1，波特率建议 `115200` 或 `460800`。
2. 将 `Debug/PIDScope/pid_debug.c` 和 `Debug/PIDScope/pid_scope_adapter.c` 加入 Keil 工程。
3. 实现 `pid_scope_port_write()`，用 USART DMA/中断/阻塞发送均可。
4. 在串口接收中断或轮询中调用 `pid_scope_rx_byte(byte)`。
5. 设置 `CAR_ENABLE_PID_SCOPE=1U`，重新编译。
