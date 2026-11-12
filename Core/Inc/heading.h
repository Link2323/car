#ifndef HEADING_H
#define HEADING_H

#include "main.h"
#include "mpu6050.h"

/* 相邻样本间隔超过该值时丢弃积分、只刷新时间基准：
   防止调试器暂停或总线长时间阻塞后，一份样本按异常长的 dt 一次性灌入错误角度。 */
#define HEADING_MAX_INTERVAL_MS 200U

/* 陀螺仪角速度积分得到的姿态角估计；角度单位度，角速度单位 °/s。
   未经加速度计等传感器融合修正，角度会随零偏残余和积分误差缓慢漂移。 */
typedef struct
{
  float x_dps;
  float y_dps;
  float z_dps;
  float x_deg;
  float y_deg;
  float z_deg;
  uint32_t sample_count;   /* 已接受的样本数，含首份基准样本 */
  uint32_t last_sample_ms; /* 最近一次接受样本时的毫秒时间戳 */
  uint8_t valid;           /* 已建立积分基准（收到过至少一份样本） */
} Heading_Data;

/* 清零全部角度、角速度快照和统计；下一份样本只重建时间基准。 */
void Heading_Reset(void);

/* 仅在 MPU6050_ReadGyro 返回 HAL_OK 后调用，传入该份角速度和读取完成时刻的
   HAL_GetTick 毫秒值；首份样本建立基准不积分，之后按相邻样本真实时间差
   累加角度。内部无总线访问，主机测试可直接调用。 */
void Heading_Update(const MPU6050_Gyro *gyro, uint32_t timestamp_ms);

/* 返回内部快照的只读指针，数据始终可读；valid=0 时角度字段无意义。 */
const Heading_Data *Heading_GetData(void);

#endif
