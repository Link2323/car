#ifndef TURN_CONTROL_H
#define TURN_CONTROL_H

#include "main.h"

/* 转向控制器状态；FAULT 表示超时或陀螺数据失效，需 Stop 后才能重新启动。 */
typedef enum
{
  TURN_IDLE = 0,
  TURN_RUNNING,
  TURN_DONE,
  TURN_FAULT
} TurnControl_State;

typedef struct
{
  TurnControl_State state;
  float target_deg;          /* 本次转向的绝对目标 z 角度（度）。 */
  float error_deg;           /* 最近一次计算的剩余角度误差（度）。 */
  float speed_counts_per_s;  /* 最近一次输出的差速轮速请求（counts/s）。 */
  uint32_t elapsed_ms;       /* 启动以来经过的毫秒数，仅调试观察。 */
} TurnControl_Data;

/* SpeedControl_Init 成功后调用一次；无硬件访问，主机测试可直接使用。
   参数固定在模块内部，实测定型后再开放为可配置结构。 */
HAL_StatusTypeDef TurnControl_Init(void);

/* 以当前 z 积分角度为基准，启动一次相对转向：角度正值左转（俯视逆时针，
   z 增大方向）、负值右转，范围 -360~360 且非零；180 度转用 +180（左转）。
   通过速度环使能电机并立即下发差速目标；转向期间不要求串口心跳，
   由到位判定、超时和陀螺数据失效三重条件自动停机。仅在主循环调用。 */
HAL_StatusTypeDef TurnControl_Start(float delta_deg);

/* 任意状态下立即停止：撤销速度环输出并回到待机，清除本次转向目标。 */
void TurnControl_Stop(void);

/* 每轮主循环调用：读取 z 角度计算误差，比例控制输出差速轮速
   （左轮 -v、右轮 +v，v 正值对应左转）；误差进入 2 度容差并保持
   300 ms 判定到位；超时或陀螺数据失效（无效或 200 ms 无新样本）时
   停机并锁存 FAULT。仅在主循环调用。 */
void TurnControl_Process(void);

/* 返回 1 表示转向进行中（速度环正被转向目标占用），用于互斥其他运动命令。 */
uint8_t TurnControl_IsActive(void);

/* 复制内部快照供调试与遥测读取，不触发控制或硬件访问。 */
HAL_StatusTypeDef TurnControl_GetData(TurnControl_Data *data);

#endif
