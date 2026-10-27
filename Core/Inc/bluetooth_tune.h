#ifndef BLUETOOTH_TUNE_H
#define BLUETOOTH_TUNE_H

#include "main.h"

/* 启动蓝牙调试接收；控制器与 USART2 必须已初始化，上电不自动运行。 */
HAL_StatusTypeDef BluetoothTune_Init(void);
/* 每轮主循环调用，解析命令、回复数值帧，并检查心跳及定时停机。
   心跳 1500 ms 超时停机，单次试转最多 30 秒；不在中断中操作电机。 */
void BluetoothTune_Process(void);
/* 回复优先于周期速度帧，避免配置回读在串口忙时被直接丢弃。 */
uint8_t BluetoothTune_CanSendTelemetry(void);
/* 主循环发送累计编码计数：标定时 200 ms，正常调试时 1000 ms。
   标定模式锁住启动入口并保持电机禁用；只通过显式退出命令解除。 */
void BluetoothTune_EncoderTelemetry(void);

#endif
