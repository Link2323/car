#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* 角速度单位均为 °/s，方向遵循传感器自身坐标系的右手定则。 */
typedef struct
{
  float x_dps;
  float y_dps;
  float z_dps;
} MPU6050_Gyro;

/* 两个候选地址的探测结果；数组下标 0/1 分别对应 0x68/0x69。
   只有 status 为 HAL_OK 时，who_am_i 才表示实际读取到的身份。 */
typedef struct
{
  HAL_StatusTypeDef status[2];
  uint8_t who_am_i[2];
} MPU6050_ProbeInfo;

/* 返回最近初始化时的身份探测结果，只读使用，不发起新的总线事务。 */
const MPU6050_ProbeInfo *MPU6050_GetProbeInfo(void);

/* 使用已初始化的 I2C1，自动识别 0x68/0x69 地址。
   支持 MPU6050(ID=0x68) 和 MPU6500(ID=0x70)，配置 ±1000 °/s、200 Hz。
   DLPF=3：6050 的带宽为 42 Hz，6500 的带宽为 41 Hz。 */
HAL_StatusTypeDef MPU6050_Init(void);

/* 阻塞采集静止零偏；采集期间必须保持静止，只在全部采样成功后更新零偏。
   samples 范围为 1~2000，每份样本约需 5 ms；不允许在中断中调用。 */
HAL_StatusTypeDef MPU6050_CalibrateGyro(uint16_t samples);

/* 读取已减去零偏的三轴角速度；HAL_BUSY 表示尚无新样本。
   只有 HAL_OK 才会更新输出；阻塞 I2C 操作仅允许在主循环中调用。 */
HAL_StatusTypeDef MPU6050_ReadGyro(MPU6050_Gyro *gyro);

#endif
