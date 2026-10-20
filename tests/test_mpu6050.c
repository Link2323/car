/* 主机回归测试：模拟 HAL 总线，不需要连接传感器。 */
#include "mpu6050.h"
#include "i2c.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
static uint32_t ticks;
static uint16_t sensor_address = 0x68U << 1;
static uint8_t who_am_i = 0x68U;
static uint8_t ready = 1U;
static uint8_t fail_read;
static uint8_t fail_write;
static uint8_t config_written;
static uint8_t data_ready_enabled;
static int16_t sample[3];

/* 返回模拟系统节拍，用于检查校准的总超时。 */
uint32_t HAL_GetTick(void)
{
  return ticks;
}

/* 直接推进模拟时间，避免测试实际等待。 */
void HAL_Delay(uint32_t delay)
{
  ticks += delay;
}

/* 模拟身份、数据就绪和三轴原始值，并校验 HAL 地址格式。 */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  unsigned int axis;
  assert(bus == &hi2c1 && reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 20U);
  if (address != sensor_address || fail_read != 0U)
  {
    return HAL_ERROR;
  }
  if (reg == 0x75U && length == 1U)
  {
    data[0] = who_am_i;
  }
  else if (reg == 0x3AU && length == 1U)
  {
    /* 复位后数据就绪中断默认关闭，状态位仅在使能后产生。 */
    data[0] = (data_ready_enabled != 0U) ? ready : 0U;
  }
  else
  {
    assert(reg == 0x43U && length == 6U);
    /* 将有符号样本编码为传感器高字节在前的数据格式。 */
    for (axis = 0U; axis < 3U; ++axis)
    {
      uint16_t value = (uint16_t)sample[axis];
      data[axis * 2U] = (uint8_t)(value >> 8);
      data[axis * 2U + 1U] = (uint8_t)value;
    }
  }
  return HAL_OK;
}

/* 校验初始化的寄存器值，并支持注入配置写入故障。 */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == &hi2c1 && address == sensor_address);
  assert(reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 20U);
  if (fail_write != 0U)
  {
    return HAL_TIMEOUT;
  }
  if (reg == 0x19U)
  {
    assert(length == 3U && data[0] == 4U && data[1] == 3U && data[2] == 0x10U);
    config_written = 1U;
  }
  else if (reg == 0x38U)
  {
    assert(length == 1U && data[0] == 0x01U);
    data_ready_enabled = 1U;
  }
  else
  {
    assert(reg == 0x6BU && length == 1U);
    assert(data[0] == 0x80U || data[0] == 0x01U);
    if (data[0] == 0x80U)
    {
      data_ready_enabled = 0U;
    }
  }
  return HAL_OK;
}

/* 检查换算误差，容许单精度浮点的正常舍入。 */
static void CheckNear(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.001f);
}

/* 覆盖地址识别、补码边界、零偏提交、数据失效和异常超时。 */
int main(void)
{
  MPU6050_Gyro gyro = {11.0f, 22.0f, 33.0f};
  assert(MPU6050_ReadGyro(&gyro) == HAL_ERROR);
  assert(MPU6050_Init() == HAL_OK && config_written != 0U);
  assert(MPU6050_ReadGyro(NULL) == HAL_ERROR);
  sample[0] = 3280;
  sample[1] = -3280;
  sample[2] = -32768;
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.x_dps, 100.0f);
  CheckNear(gyro.y_dps, -100.0f);
  CheckNear(gyro.z_dps, -32768.0f / 32.8f);
  sample[2] = 32767;
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.z_dps, 32767.0f / 32.8f);

  assert(MPU6050_CalibrateGyro(0U) == HAL_ERROR);
  assert(MPU6050_CalibrateGyro(2001U) == HAL_ERROR);
  assert(MPU6050_CalibrateGyro(200U) == HAL_OK);
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.x_dps, 0.0f);
  CheckNear(gyro.y_dps, 0.0f);
  CheckNear(gyro.z_dps, 0.0f);

  ready = 0U;
  gyro.z_dps = 123.0f;
  assert(MPU6050_ReadGyro(&gyro) == HAL_BUSY && gyro.z_dps == 123.0f);
  assert(MPU6050_CalibrateGyro(2U) == HAL_TIMEOUT);
  ready = 1U;
  fail_read = 1U;
  assert(MPU6050_CalibrateGyro(2U) == HAL_ERROR);
  assert(MPU6050_ReadGyro(&gyro) == HAL_ERROR && gyro.z_dps == 123.0f);
  fail_read = 0U;
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.z_dps, 0.0f);

  /* AD0 接高时必须识别 0x69；重新初始化应清除旧零偏。 */
  sensor_address = 0x69U << 1;
  assert(MPU6050_Init() == HAL_OK);
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.x_dps, 100.0f);
  who_am_i = 0x70U;
  /* 实机返回 MPU6500 身份 0x70：必须走完初始化、校准和角速度读取。 */
  assert(MPU6050_Init() == HAL_OK);
  assert(MPU6050_GetProbeInfo()->who_am_i[1] == 0x70U);
  assert(MPU6050_CalibrateGyro(200U) == HAL_OK);
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.z_dps, 0.0f);
  sample[2] -= 3280;
  assert(MPU6050_ReadGyro(&gyro) == HAL_OK);
  CheckNear(gyro.z_dps, -100.0f);
  who_am_i = 0x7FU;
  assert(MPU6050_Init() == HAL_ERROR);
  assert(MPU6050_ReadGyro(&gyro) == HAL_ERROR);
  who_am_i = 0x68U;
  fail_write = 1U;
  assert(MPU6050_Init() == HAL_TIMEOUT);
  assert(MPU6050_ReadGyro(&gyro) == HAL_ERROR);
  puts("MPU6050 tests passed");
  return 0;
}
