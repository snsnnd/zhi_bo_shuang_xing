# Debug

调试、日志和上位机通信适配目录。

- `PIDScope/`：PIDScope 二进制波形调参协议、CRC、遥测缓存和 PID 参数写入适配。

文本 CMD 命令的具体语义仍放在 `Control/remote_control.*`，因为它会直接影响整车运行状态；`Debug/` 只放调试协议、遥测和日志类能力。
