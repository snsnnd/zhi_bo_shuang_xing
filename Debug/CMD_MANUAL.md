# HC-05 CMD 使用说明

本文说明通过 HC-05 蓝牙串口控制小车的文本命令。命令解析代码在 `Control/remote_control.*`，串口收发由 `BSP/bsp_hc05.*` 提供。

## 连接参数

- 蓝牙模块：HC-05。
- MCU 串口：USART2，PA2=TX，PA3=RX。
- 默认波特率：`9600`，由 `HC05_DEFAULT_BAUD` 定义。
- 上位机端口：选择 HC-05 的蓝牙串口 `Outgoing` COM。
- 命令结束符：`\r`、`\n`、`\r\n` 均可。
- 命令大小写：不敏感，`ping` 和 `PING` 等价。
- 兼容格式：可带 `$` 前缀，例如 `$PING`。

## 安全开关

- `CAR_ENABLE_HC05=1U`：启用 HC-05 字节通道。
- `CAR_ENABLE_REMOTE_CONTROL=1U`：启用 CMD 控制。
- `CAR_ENABLE_MOTOR_OUTPUT=0U`：电机输出能力不会编译进固件，即使发送 `START` 或 `MOTOR 1` 也不会真实输出。
- `CAR_ENABLE_MOTOR_OUTPUT=1U` 只是编译进能力；上电默认仍关闭，需要确认实车安全后再执行 `CFG ARM MOTOR_OUTPUT` 和 `CFG SET MOTOR_OUTPUT 1`。
- 超过 `REMOTE_CONTROL_TIMEOUT_MS` 未收到命令时，会自动禁止电机并进入强制停车状态。

## 快速测试

发送：

```text
PING
```

正常回复：

```text
$OK,PONG
```

查询状态：

```text
STATUS
```

回复示例：

```text
$STATUS,MOTOR,0,STOP,1,SPEED,0.000,LIMIT,0.250,MODE,3
```

## 命令列表

| 命令 | 参数 | 作用 | 回复 |
| --- | --- | --- | --- |
| `HELP` | 无 | 返回命令摘要 | `$HELP ...` |
| `PING` | 无 | 通信链路测试 | `$OK,PONG` |
| `START` | 无 | 允许运行，清除强制停车 | `$OK,START` |
| `RUN` | 无 | `START` 的别名 | `$OK,START` |
| `STOP` | 无 | 禁止电机并强制停车 | `$OK,STOP` |
| `AUTO` | 无 | 取消速度覆盖和模式覆盖，恢复自动策略 | `$OK,AUTO` |
| `MOTOR` | `0/1`、`ON/OFF`、`ENABLE` | 禁止或允许电机输出 | `$OK,MOTOR,0/1` |
| `SPEED` | `<mps>` | 覆盖目标速度，单位 m/s | `$OK,SPEED` |
| `LIMIT` | `<mps>` | 设置运行时速度上限 | `$OK,LIMIT` |
| `MAXSPEED` | `<mps>` | `LIMIT` 的别名 | `$OK,LIMIT` |
| `CFG` | `?` | 查询关键运行态配置摘要 | `$CFG,...` |
| `CFG` | `LIST` | 逐行列出所有运行期开关状态 | `$CFG,FEATURE,...` |
| `CFG` | `TIMEOUT <ms>` | 修改远控超时时间，范围 100-10000ms | `$OK,CFG,TIMEOUT` |
| `CFG` | `LIMIT <mps>` | 修改运行时速度上限 | `$OK,CFG,LIMIT` |
| `CFG` | `SPEED <mps>` | 修改目标速度 | `$OK,CFG,SPEED` |
| `CFG` | `GET <feature>` | 查询单个运行期开关 | `$CFG,FEATURE,...` |
| `CFG` | `SET <feature> <0/1>` | 修改运行期开关 | `$OK,CFG,SET` |
| `CFG` | `ARM <feature>` | 对安全开关做第一次确认 | `$OK,CFG,ARM` |
| `MODE` | `<name>` | 覆盖小车模式 | `$OK,MODE` |
| `PID` | `<target> <kp> <ki> <kd>` | 在线修改 PID 参数 | `$OK,PID` |
| `STATUS` | 无 | 查询状态 | `$STATUS,...` |
| `STATUS?` | 无 | `STATUS` 的别名 | `$STATUS,...` |

## MODE 参数

`MODE` 支持：`IDLE`、`CALIB`、`CALIBRATION`、`LEARN`、`LEARNING`、`TRACK`、`TRACKING`、`PREDICT`、`PREDICTION`、`HAIRPIN`、`LOST`、`PROTECT`。

示例：

```text
MODE TRACKING
MODE PROTECT
```

## 速度控制

设置目标速度：

```text
SPEED 0.15
```

设置速度上限：

```text
LIMIT 0.20
```

说明：

- 速度单位为 m/s。
- `SPEED` 会被限制在 `0` 到当前 `speed_limit_mps`。
- `LIMIT` 最大不超过 `SPEED_HIGH_MPS`。
- 电机输出能力未编译或运行期 `MOTOR_OUTPUT=0` 时，速度命令只改变运行状态，不会驱动电机。

## 配置查询和运行时配置

查询当前关键配置：

```text
CFG ?
```

回复示例：

```text
$CFG,HC05,1,REMOTE,1,MOTOR_OUT,0,SPEED_PID,0,PID_SCOPE,0,PID_HC05,0,OLED,1,LINE,1,IMU,1,ENC,1,TIMEOUT,1000,LIMIT,0.250,SPEED,0.000
```

列出全部运行期开关：

```text
CFG LIST
```

回复示例：

```text
$CFG,FEATURE,MOTOR_OUTPUT,0,ARM,1
$CFG,FEATURE,SPEED_PID,0,ARM,1
$OK,CFG,LIST
```

修改远控超时：

```text
CFG TIMEOUT 3000
```

修改速度上限：

```text
CFG LIMIT 0.20
```

修改目标速度：

```text
CFG SPEED 0.12
```

说明：

- `CFG` 只能修改运行时变量，不能打开未编译进固件的能力。
- 当前固件已经把主要功能编译进程序，`CFG SET` 修改的是运行期开关。
- 安全开关需要双重确认：先 `CFG ARM <feature>`，5 秒内再 `CFG SET <feature> 1`。
- 安全开关包括：`MOTOR_OUTPUT`、`SPEED_PID`、`TRACK_MAP_LEARNING`。
- 关闭安全开关不需要 arm，例如 `CFG SET MOTOR_OUTPUT 0`。

可用 feature 名称：

- `LINE_SENSOR`
- `IMU`
- `ENCODER`
- `OLED`
- `MOTOR_OUTPUT`
- `SPEED_PID`
- `TRACK_MAP_LEARNING`
- `MAP_PREDICTION`
- `LOST_PROTECTION`
- `PID_SCOPE`
- `HC05`
- `PID_SCOPE_OVER_HC05`
- `REMOTE_CONTROL`
- `HC05_OLED_TEST`
- `HC05_RX_ECHO_TEST`

示例：

```text
CFG GET MOTOR_OUTPUT
CFG ARM MOTOR_OUTPUT
CFG SET MOTOR_OUTPUT 1
CFG SET MOTOR_OUTPUT 0
```

## 电机启停

允许电机：

```text
MOTOR 1
MOTOR ON
START
```

禁止电机或停车：

```text
MOTOR 0
STOP
```

恢复自动策略：

```text
AUTO
```

建议实车测试顺序：`PING`、`STATUS`、`MOTOR 0`、`SPEED 0.10`、确认车辆架空、`CFG ARM MOTOR_OUTPUT`、`CFG SET MOTOR_OUTPUT 1`、`MOTOR 1`、`START`。

## PID 在线调参

命令格式：

```text
PID <target> <kp> <ki> <kd>
```

`target` 支持：

- `LINE`：循迹转向 PID。
- `LEFT`、`LSPD`、`SPEED_LEFT`：左轮速度 PID。
- `RIGHT`、`RSPD`、`SPEED_RIGHT`：右轮速度 PID。
- `SPEED`、`BOTH`、`SPEED_BOTH`：左右轮速度 PID 同时修改。

示例：

```text
PID LINE 120 0 25
PID SPEED 500 50 0
```

说明：

- PID 命令只修改 RAM 中的运行参数，掉电不保存。
- 修改 PID 后会清空对应 PID 的积分和历史状态，避免旧状态造成冲击。
- 当前 `CAR_ENABLE_SPEED_PID=0U` 时，速度 PID 参数可被解析，但速度闭环未启用，不会影响输出。

## 常见问题

### 发了命令没有回复

- 确认选择 HC-05 的 `Outgoing` COM。
- 确认波特率和固件 `HC05_DEFAULT_BAUD` 一致；当前默认是 `9600`。
- 确认命令带换行结束。
- 确认接线：HC-05 TXD -> PA3，HC-05 RXD -> PA2，GND 共地。

### 能收到数据但控制不了电机

- 检查 `CAR_ENABLE_MOTOR_OUTPUT` 是否编译进固件，并确认已执行 `CFG ARM MOTOR_OUTPUT` 和 `CFG SET MOTOR_OUTPUT 1`。
- 检查是否发送了 `MOTOR 1` 和 `START`。
- 检查是否设置了非零速度，例如 `SPEED 0.10`。
- 检查 `STATUS` 里的 `STOP` 是否为 `1`。
- 检查是否超时触发了远控保护，需要持续发送命令或重新 `START`。

### PIDScope 和 CMD 能否共用蓝牙

可以。PIDScope 二进制协议和 CMD 文本命令共用同一个 HC-05 字节通道。调试蓝牙基础链路时建议保持运行期开关 `PID_SCOPE=0`，确认 CMD 正常后再 `CFG SET PID_SCOPE 1` 和 `CFG SET PID_SCOPE_OVER_HC05 1` 开启波形调参。
