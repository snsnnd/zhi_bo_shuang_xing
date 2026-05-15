# Core

STM32CubeMX 生成的工程核心目录。

- `Inc/`：外设初始化头文件、中断头文件和 HAL 配置。
- `Src/`：`main.c`、GPIO、TIM、I2C、MSP、中断和系统时钟代码。

除 `USER CODE BEGIN/END` 区域外，不建议手工修改本目录生成代码；重新生成 CubeMX 工程可能覆盖文件。
