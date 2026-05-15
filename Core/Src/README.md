# Core/Src

CubeMX 生成的源文件目录。

这里负责系统时钟、GPIO、I2C2、TIM2/TIM3 编码器、TIM4 PWM、中断和 HAL MSP 初始化。业务逻辑不要直接写入生成区，应放到 `Application/`、`BSP/`、`Control/` 或 `Strategy/`。
