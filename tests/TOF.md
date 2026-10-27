# 三路测距验证

实物验证已通过，主程序中的电机循环试转和测距蓝牙测试输出已移除。以下蓝牙帧格式与接收脚本保留作回归验证参考，当前固件不自动发送这些帧；需要观察当前读数时可使用 ST-Link 查看快照变量。

三路 VL53L0X 共用 I2C1，GPIO 初始化保持 XSHUT 低电平，`TOF_Init()` 按前、左、右依次释放并改址。七位地址分别为前 `0x31`、左 `0x30`、右 `0x32`；XSHUT 分别为 PA5、PA4、PB5。初始化允许有界启动/校准延时，主循环读取不等待测量完成。

`TOF_Update()` 每路每 5 ms 查询一次，器件采用 33 ms 测量预算和 50 ms 连续测量周期。单路出错会关闭该路，继续服务其他路；调用 `TOF_Init()` 可整体复位重试。前方输出减去已有标定偏移 32 mm，左右默认不补偿，可分别设置 `TOF_LEFT_OFFSET_MM`、`TOF_RIGHT_OFFSET_MM`。

通过 `TOF_GetSensorData(TOF_FRONT/TOF_LEFT/TOF_RIGHT, &data)` 读取数据，只有 `valid=1` 才能用于控制；超过 200 ms 的快照失效。`TOF_GetSensorStatus()` 获取逐路 HAL 结果。ST-Link 观察 `car_tof_sensors`、`tof_sensor_status`；旧变量 `car_tof` 仍对应前方。

此前测试固件通过蓝牙每 100 ms 发送一行 FireWater CSV：

```text
tof:前距离,前有效,前状态,前原始状态,左距离,左有效,左状态,左原始状态,右距离,右有效,右状态,右原始状态
```

距离单位 mm，无效输出 -1。状态：0 正常、1 初始化失败、2 I2C 错误、3 超时、4 无效回波、5 等待首帧。正常原始状态为 11。三路最大帧长 43 字节，可在 9600 波特率下以 100 ms 周期发送。

## 主机测试

在 `software/car/` 下使用 MinGW g++ 编译并运行真实驱动与 HAL 模拟器：

```powershell
g++ -std=c++11 -DSTM32F103xB -ICore/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include tests/test_tof.cpp Core/Src/tof.cpp ThirdParty/VL53L0X/VL53L0X.cpp -o tests/test_tof.exe
./tests/test_tof.exe
```

CMSIS 在 64 位主机下会产生地址宽度警告，目标 Keil ARMCC 构建无此警告。测试覆盖三路地址冲突检测、独立距离/补偿、无效回波、轮询节流、单路通信故障隔离、初始化失败、校准超时、重初始化及节拍回绕。

## 实物验收

以下为此前测试固件的验收步骤；当前固件需恢复测试帧发送后才能使用串口脚本。Keil 打开 `MDK-ARM/car.uvprojx`，编译后按 F8 烧录。三个方向各放置量程内目标，移动其中一个目标，确认只有对应距离通道变化；左右偏移需分别实测标定。电机调试保持关闭。

```powershell
./tests/read_tof.ps1 -PortName COM3 -Seconds 15
```

脚本逐路统计有效/无效帧，要求三路均收到有效距离。拔掉一路后，检查该路 valid 变为 0 且距离输出 -1，其他两路继续刷新；共用总线的物理短路仍可能影响全部器件。重新接回后需复位或调用 `TOF_Init()`。主机模拟与编译通过不代表完成实物验收。
