#ifndef MAZE_H
#define MAZE_H

#include "main.h"

/* 迷宫探索状态机状态。FAULT 表示感知或动作异常，需 Stop 复位。 */
typedef enum
{
  MAZE_IDLE = 0,   /* 待机，等待启动命令。 */
  MAZE_DECIDE,     /* 停车感知：读三路测距并按左手法则决策。 */
  MAZE_TURNING,    /* 原地转向（由 turn_control 执行），等待完成。 */
  MAZE_DRIVING,    /* 直行：航向保持 + 事件监测（前墙/左口/超时）。 */
  MAZE_CREEPING,   /* 左口确认后的短滑行：走到开口中部再停车决策。 */
  MAZE_FAULT       /* 保护停机：感知无效或驱动超时。 */
} Maze_State;

/* 决策结果，同时用于遥测观察。 */
typedef enum
{
  MAZE_DECISION_NONE = 0,
  MAZE_DECISION_LEFT,
  MAZE_DECISION_STRAIGHT,
  MAZE_DECISION_RIGHT,
  MAZE_DECISION_UTURN
} Maze_Decision;

typedef struct
{
  Maze_State state;
  Maze_Decision decision;       /* 最近一次格口决策。 */
  uint16_t left_mm;             /* 决策时刻的三路距离快照，mm。 */
  uint16_t front_mm;
  uint16_t right_mm;
  uint8_t sensor_valid;         /* 位 0/1/2 = 前/左/右实测有效；前路无回波探测时可为 6。 */
  float heading_target_deg;     /* 当前直行段的航向基准（度）。 */
  uint32_t drive_elapsed_ms;    /* 当前直行段已行驶时间，仅观察。 */
  uint32_t steps;               /* 已完成的决策次数。 */
  uint8_t fault_reason;         /* 0 无；1 感知无效；2 速度环故障；
                                   3 使能失败；4 陀螺仪数据失效。 */
} Maze_Data;

/* TurnControl/SpeedControl/TOF 初始化成功后调用一次；重复调用返回忙。 */
HAL_StatusTypeDef Maze_Init(void);

/* 启动左手法则探索：要求陀螺仪积分有效且全部模块待机。
   立即进入第一次停车决策；探索期间不要求串口心跳，由感知保护、
   直行超时和转向自带的三重保护兜底。仅在主循环调用。 */
HAL_StatusTypeDef Maze_Start(void);

/* 停止探索并撤销电机输出，回到待机；同时清除故障锁存。 */
void Maze_Stop(void);

/* 每轮主循环调用：驱动 DECIDE→TURNING→DRIVING 循环。
   DRIVING 直行时以该段起始 z 角度为基准做比例差速纠偏，并监测：
   前方距离小于停车阈值、左侧出现开口（均需连续确认）、或直行超时，
   触发后停车回到 DECIDE。前路通信正常且新鲜的指定无回波状态允许
   每轮至多约 20 mm/s 探测；已见近墙不会被无回波清除。
   通信故障、超时、旧数据、其他前向错误和左右无效仍保护停机。 */
void Maze_Process(void);

/* 返回 1 表示探索进行中，用于互斥其他运动命令。 */
uint8_t Maze_IsActive(void);

/* 复制内部快照供遥测与调试，不触发控制或硬件访问。 */
HAL_StatusTypeDef Maze_GetData(Maze_Data *data);

#endif
