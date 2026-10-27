#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

/* 实测两轮前进时：左侧 TIM3 减计数，右侧 TIM4 加计数。
   左轮取反以统一为前进为正；更换接线或安装方向后须重新手转确认。 */
#ifndef ENCODER_LEFT_SIGN
#define ENCODER_LEFT_SIGN -1
#endif
#ifndef ENCODER_RIGHT_SIGN
#define ENCODER_RIGHT_SIGN 1
#endif
#define ENCODER_SAMPLE_PERIOD_MS 10U

typedef struct
{
  int32_t left_delta;        /* 本次采样间隔内的左轮计数增量，已修正方向。 */
  int32_t right_delta;       /* 本次采样间隔内的右轮计数增量，已修正方向。 */
  int64_t left_total;        /* 从初始化或清零以来累计的左轮计数。 */
  int64_t right_total;       /* 从初始化或清零以来累计的右轮计数。 */
  float left_counts_per_s;   /* 左轮计数率，单位 counts/s，按实际采样间隔计算。 */
  float right_counts_per_s;  /* 右轮计数率，单位 counts/s，按实际采样间隔计算。 */
  uint32_t interval_ms;      /* 本次采样间隔，单位 ms。 */
  uint32_t timestamp_ms;     /* 本次读取对应的系统节拍。 */
  uint8_t sample_valid;      /* 1 为有效采样，初始化、清零或歧义采样后为 0。 */
  uint8_t total_valid;       /* 0 表示发生过计数歧义，累计位移不完整，清零后恢复。 */
} Encoder_Data;

/* 在 MX_TIM3_Init/MX_TIM4_Init 后调用一次，启动双路硬件正交计数。
   不清零硬件 CNT，以当前计数为累计计数的起点。 */
HAL_StatusTypeDef Encoder_Init(void);

/* 主循环持续调用，至少间隔 10 ms 才采样，否则返回 HAL_BUSY。
   任意两次采样间每轮实际计数位移必须严格小于 32768，否则回绕可能误判。
   恰好半个计数器范围（32768）存在歧义，返回 HAL_ERROR 并丢弃该次增量。 */
HAL_StatusTypeDef Encoder_Update(void);

/* 复制最近一次数据，不读取硬件、不消费增量；调用者应检查 sample_valid。 */
HAL_StatusTypeDef Encoder_GetData(Encoder_Data *data);

/* 将当前硬件计数设为新的软件零点，累计/增量/计数率清零，不改硬件 CNT。 */
HAL_StatusTypeDef Encoder_Reset(void);

/* 上述接口仅在主循环调用，模块独占 TIM3/TIM4 的启动和软件采样状态。 */
#endif
