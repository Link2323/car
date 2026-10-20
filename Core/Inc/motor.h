#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* 命令为有符号千分比：-1000~1000，0 表示双输入低电平、停止滑行。
   正数表示车轮前进，负数表示后退；按当前安装，左轮正数驱动 FI、右轮正数驱动 BI。
   右轮方向已按实测反转校正，更换电机接线或安装方向后需重新确认。
   本模块是开环占空比驱动，不是轮速闭环控制，所有接口仅在主循环调用。 */
#define MOTOR_DUTY_MAX 1000
#define MOTOR_START_DELAY_MS 5U
#define MOTOR_REVERSE_DELAY_MS 5U

/* 在 MX_TIM1_Init/MX_TIM2_Init 后调用一次，启动四路零占空比 PWM。
   默认禁用；TIM1 的 1 ms 基准中断仍由主程序负责启动。 */
HAL_StatusTypeDef Motor_Init(void);

/* 显式使能并清除旧命令；5 ms 后才允许输出。需先确认驱动电源已经接通。
   反复使能不会重新启动已在工作的电机，也不会重置其命令。 */
HAL_StatusTypeDef Motor_Enable(void);

/* 清除命令、停止滑行并锁定输出，之后必须再次显式使能。 */
void Motor_Disable(void);

/* 停止两轮并取消待执行的换向命令，保留使能状态。 */
void Motor_Stop(void);

/* 设置两轮目标占空比，超范围饱和；禁用时返回 HAL_BUSY，不记住旧命令。
   返回成功表示命令已接收，启动等待或换向空档期间并非立即输出目标值。 */
HAL_StatusTypeDef Motor_SetDuty(int16_t left, int16_t right);

/* 主循环持续调用，非阻塞完成启动等待与换向空档，不使用 HAL_Delay。 */
void Motor_Process(void);

#endif
