# car 固件仓库说明

迷宫小车（Micromouse）固件，用于迷宫搜索的开发调试。

## 工程结构与构建

- MCU：STM32F103C8Tx，72 MHz（HSE 8 MHz × PLL9）
- 框架：STM32CubeMX 生成（工程文件 `car.ioc`）+ Keil MDK-ARM，ARMCC V5 编译器
- 构建：Keil µVision 打开 `MDK-ARM/car.uvprojx`，唯一 target 为 `car`，编译（F7）+ 烧录（F8，ST-Link）
- 依赖库：`Drivers/`（ST HAL/CMSIS，CubeMX 生成，不要手改）、`ThirdParty/VL53L0X/`（激光测距，C++ 实现）

## 目录一览

| 路径 | 内容 |
|---|---|
| `Core/Src`、`Core/Inc` | 应用代码；CubeMX 生成框架与手写驱动模块都在这里 |
| `tests/` | 主机端单元测试（自带 HAL 桩函数）与串口观察 PowerShell 脚本 |
| `MDK-ARM/` | Keil 工程文件；编译输出目录 `car/` 已被 gitignore |

## 外设分配（car.ioc）

| 外设 | 用途 |
|---|---|
| TIM1 CH1/CH2 + TIM2 CH3/CH4 | 双电机 H 桥四路 PWM；TIM1 同时复用为 1 ms 时基中断 |
| TIM3 / TIM4 | 左右轮硬件正交编码计数 |
| I2C1（100 kHz） | 前方 VL53L0X 激光测距 + MPU6050 陀螺仪 |
| USART2（9600） | HC05 蓝牙，输出调试/绘图数据帧 |

## 软件模块（Core/ 下，均只有 .h/.c 一对）

- `motor` — 开环占空比电机驱动（非轮速闭环）。命令为千分比 -1000~1000；内部维护启动延时与换向空档状态机；默认禁用，需显式 `Motor_Enable()`。右轮方向已按实测反转校正。
- `encoder` — 编码器采样，10 ms 周期；输出增量/累计/计数率三组数据，含 16 位回绕修正。任意两次采样间每轮计数必须小于 32768。
- `tof` — 前方 VL53L0X（改址 0x31），5 ms 轮询非阻塞测距；输出含 -32 mm 偏移补偿（仅适用当前模块）与诊断状态机（TOF_State）。
- `mpu6050` — 陀螺仪，自动探测 0x68/0x69，兼容 MPU6500；上电需静止校准零偏（`MPU6050_CalibrateGyro`）。
- `hc05` — USART2 中断发送字符串（≤127 字节），忙时丢弃，模块独占 USART2 发送。

## 架构约定（改代码前必读）

- **非阻塞主循环**：所有模块接口只允许在主循环调用；I2C 等阻塞操作仅在主循环执行，中断里只累计节拍。`Motor_Process`/`TOF_Update`/`Encoder_Update` 均为轮询式状态机，不使用 `HAL_Delay`。
- **CubeMX 重新生成会覆盖 USER CODE 区之外的代码**：手写代码必须放在 `/* USER CODE BEGIN/END */` 之间；新增模块用独立的 .c/.h 放 `Core/`。
- **调试可见性**：各阶段 `HAL_StatusTypeDef` 保存到全局变量（如 `tof_status`），供 ST-Link 调试器与蓝牙帧观察失败位置；串口只输出 FireWater 数值帧（如 `tof:距离,有效,状态,原始状态\n`），不混入文字。
- **电机安全**：故障进 `Error_Handler` 前先 `Motor_Disable()`；测试性动作通过 `main.c` 顶部的 `MOTOR_DEBUG_*` 开关控制，默认关闭。

## 编码规范

- 注释一律中文，且要充分：每个函数说明职责，关键循环和条件判断解释目的与含义（继承上层 AGENTS.md 的要求）。
- 模块对外接口在 .h 中以中文注释文档化（现有模块均如此，保持风格一致）。
- 命名沿用现有惯例：模块前缀 + 动词（`Motor_SetDuty`、`TOF_Update`），全局变量蛇形命名。

## 测试

- `tests/read_com6.ps1` / `read_tof.ps1`：Windows PowerShell 串口接收脚本（9600 波特率），配合蓝牙串口检查输出帧格式与故障率。
- 调试方式：ST-Link 烧录和在线调试修改代码，蓝牙实时回读数据帧，两者配合无需拆线即可快速迭代调试。
