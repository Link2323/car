#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "encoder.h"

/* 单位：速度 counts/s、输出千分比、时间秒；轮径和每圈计数未知时不换算 mm/s。 */
#define SPEED_CONTROL_FEEDBACK_TIMEOUT_MS 50U

typedef enum
{
  SPEED_CONTROL_IDLE = 0,
  SPEED_CONTROL_RUNNING,
  SPEED_CONTROL_FAULT
} SpeedControl_State;

typedef enum
{
  SPEED_CONTROL_FAULT_NONE = 0,
  SPEED_CONTROL_FAULT_FEEDBACK,
  SPEED_CONTROL_FAULT_MOTOR,
  SPEED_CONTROL_FAULT_COMMAND
} SpeedControl_Fault;

typedef struct
{
  float kp;                 /* 比例增益，单位：千分比 / (counts/s)。 */
  float ki;                 /* 积分增益，单位：千分比 / count。 */
  float kd;                 /* 测量微分增益，默认零，避免低速量化噪声放大。 */
} SpeedControl_Gains;

typedef struct
{
  SpeedControl_Gains left;
  SpeedControl_Gains right;
  float max_target_counts_per_s;
  int16_t max_duty;          /* 同时限制输出和积分项，范围 1~1000。 */
} SpeedControl_Config;

typedef struct
{
  SpeedControl_State state;
  SpeedControl_Fault fault;
  float left_target;
  float right_target;
  float left_measured;
  float right_measured;
  int16_t left_duty;         /* 控制器请求值；换向空档中不代表实际 PWM。 */
  int16_t right_duty;
  uint32_t feedback_timestamp_ms;
} SpeedControl_Data;

/* Motor_Init/Encoder_Init 成功后调用；复制并校验配置，始终以禁用状态启动。
   重复初始化返回 HAL_BUSY；运行时调参使用 Configure 接口。 */
HAL_StatusTypeDef SpeedControl_Init(const SpeedControl_Config *config);
/* 完整校验后原子更新双轮配置；保留并限幅积分，Ki 为零时清除积分。
   运行中降低限幅会立即约束目标和请求输出，重置微分历史避免突跳。
   新增益在下一份有效采样生效；无效配置不改变任何现有参数。 */
HAL_StatusTypeDef SpeedControl_Configure(const SpeedControl_Config *config);
/* 读取实际生效的配置，供蓝牙回读；不访问硬件。 */
HAL_StatusTypeDef SpeedControl_GetConfig(SpeedControl_Config *config);
/* 必须已有新鲜有效的编码器采样；使能后目标为零，不恢复旧命令。
   故障锁存后必须先调用 Stop，再显式 Enable；所有接口仅在主循环调用。 */
HAL_StatusTypeDef SpeedControl_Enable(void);
/* 撤销输出、清除双轮积分和目标，并清除故障锁存，返回待机状态。 */
void SpeedControl_Stop(void);
/* 设置双轮目标，超范围限幅；NaN/无穷大返回错误并在运行时停机。
   单轮目标为零立即撤销该轮输出；目标换向时先清除该轮积分和输出。 */
HAL_StatusTypeDef SpeedControl_SetTarget(float left, float right);
/* 每轮主循环调用，且耗时 I2C 返回后、Encoder_Update 之前再调用一次。
   只对新采样计算 PID；无效、过期或间隔 >=50 ms 的反馈触发锁存停机。
   此软件保护只能在主循环恢复执行时生效，不能中断正在阻塞的 I2C。 */
HAL_StatusTypeDef SpeedControl_Update(void);
/* 复制状态供调试与上层应用读取，不触发控制或硬件访问。 */
HAL_StatusTypeDef SpeedControl_GetData(SpeedControl_Data *data);

#endif
