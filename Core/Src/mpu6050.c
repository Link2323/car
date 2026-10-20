#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_INT_STATUS    0x3AU
#define MPU6050_REG_GYRO_XOUT_H   0x43U
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_I2C_TIMEOUT_MS    20U
#define MPU6050_GYRO_SCALE       32.8f
#define MPU6050_DEVICE_ID        0x68U
#define MPU6500_DEVICE_ID        0x70U

static uint16_t device_address;
static uint8_t initialized;
static float gyro_bias[3];
static MPU6050_ProbeInfo probe_info = {{HAL_ERROR, HAL_ERROR}, {0U, 0U}};

/* 暴露地址和身份探测结果，避免最后一次失败覆盖先前地址的有效诊断。 */
const MPU6050_ProbeInfo *MPU6050_GetProbeInfo(void)
{
  return &probe_info;
}

/* 向当前器件写一个寄存器；STM32 HAL 使用左移一位的总线地址。 */
static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, device_address, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           MPU6050_I2C_TIMEOUT_MS);
}

/* 读取连续寄存器，限制单次总线等待时间，避免传感器断线时永久阻塞。 */
static HAL_StatusTypeDef MPU6050_ReadRegisters(uint8_t reg, uint8_t *data,
                                              uint16_t length)
{
  return HAL_I2C_Mem_Read(&hi2c1, device_address, reg,
                          I2C_MEMADD_SIZE_8BIT, data, length,
                          MPU6050_I2C_TIMEOUT_MS);
}

/* 按高字节在前合并二进制补码，显式处理负值，避免依赖有符号窄化转换。 */
static int32_t MPU6050_DecodeRaw(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 8) | data[1];
  return (value >= 32768U) ? (int32_t)value - 65536 : (int32_t)value;
}

/* 检查新数据并一次突发读取三轴，保证同一份输出的三轴来自同一次采样。 */
static HAL_StatusTypeDef MPU6050_ReadRaw(int32_t raw[3])
{
  uint8_t status;
  uint8_t data[6];
  HAL_StatusTypeDef result;

  result = MPU6050_ReadRegisters(MPU6050_REG_INT_STATUS, &status, 1U);
  if (result != HAL_OK)
  {
    return result;
  }
  /* DATA_RDY 未置位时不重复消费旧样本，校准也使用这一判断。 */
  if ((status & 0x01U) == 0U)
  {
    return HAL_BUSY;
  }
  result = MPU6050_ReadRegisters(MPU6050_REG_GYRO_XOUT_H, data, 6U);
  if (result == HAL_OK)
  {
    raw[0] = MPU6050_DecodeRaw(&data[0]);
    raw[1] = MPU6050_DecodeRaw(&data[2]);
    raw[2] = MPU6050_DecodeRaw(&data[4]);
  }
  return result;
}

/* 识别芯片、复位并配置陀螺仪；初始化失败后禁止继续读取旧配置下的数据。 */
HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t id = 0U;
  uint8_t address;
  /* 两型号的这些陀螺仪寄存器兼容：1 kHz / (4+1)，DLPF=3，FS_SEL=2。
     MPU6500 的 FCHOICE_B 保持 00，启用 DLPF，带宽为 41 Hz；6050 为 42 Hz。 */
  uint8_t config[3] = {4U, 3U, 0x10U};
  HAL_StatusTypeDef result;

  initialized = 0U;
  probe_info.status[0] = HAL_ERROR;
  probe_info.status[1] = HAL_ERROR;
  probe_info.who_am_i[0] = 0U;
  probe_info.who_am_i[1] = 0U;
  /* AD0 决定总线地址，与身份寄存器独立；只接受明确支持的两种芯片 ID。 */
  for (address = 0x68U; address <= 0x69U; ++address)
  {
    device_address = (uint16_t)address << 1;
    id = 0U;
    probe_info.status[address - 0x68U] =
      MPU6050_ReadRegisters(MPU6050_REG_WHO_AM_I, &id, 1U);
    probe_info.who_am_i[address - 0x68U] = id;
    if (probe_info.status[address - 0x68U] == HAL_OK &&
        (id == MPU6050_DEVICE_ID || id == MPU6500_DEVICE_ID))
    {
      break;
    }
  }
  if (address > 0x69U)
  {
    return HAL_ERROR;
  }
  result = MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x80U);
  if (result != HAL_OK)
  {
    return result;
  }
  HAL_Delay(100U);
  /* 清除休眠并选用 X 轴陀螺仪 PLL 时钟，降低采样时钟漂移。 */
  result = MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01U);
  if (result != HAL_OK)
  {
    return result;
  }
  result = HAL_I2C_Mem_Write(&hi2c1, device_address, MPU6050_REG_SMPLRT_DIV,
                             I2C_MEMADD_SIZE_8BIT, config, sizeof(config),
                             MPU6050_I2C_TIMEOUT_MS);
  if (result != HAL_OK)
  {
    return result;
  }
  /* INT_STATUS 的 DATA_RDY_INT 依赖数据就绪中断使能。
     即使不连接 INT 引脚、只轮询状态寄存器，也必须打开此中断源。 */
  result = MPU6050_WriteRegister(MPU6050_REG_INT_ENABLE, 0x01U);
  if (result != HAL_OK)
  {
    return result;
  }
  /* 等待陀螺仪启动和低通滤波器稳定，再允许校准。 */
  HAL_Delay(100U);
  gyro_bias[0] = 0.0f;
  gyro_bias[1] = 0.0f;
  gyro_bias[2] = 0.0f;
  initialized = 1U;
  return HAL_OK;
}

/* 在静止条件下对多份新样本求平均；出错保留先前零偏，避免半成品校准。 */
HAL_StatusTypeDef MPU6050_CalibrateGyro(uint16_t samples)
{
  int32_t sum[3] = {0, 0, 0};
  int32_t raw[3];
  uint16_t count = 0U;
  uint32_t started;
  HAL_StatusTypeDef result;

  if (initialized == 0U || samples == 0U || samples > 2000U)
  {
    return HAL_ERROR;
  }
  started = HAL_GetTick();
  /* 每次只累计新样本；总超时也覆盖器件应答但一直不产生数据的情况。 */
  while (count < samples)
  {
    if ((uint32_t)(HAL_GetTick() - started) >= (uint32_t)samples * 10U + 100U)
    {
      return HAL_TIMEOUT;
    }
    result = MPU6050_ReadRaw(raw);
    if (result == HAL_OK)
    {
      sum[0] += raw[0];
      sum[1] += raw[1];
      sum[2] += raw[2];
      ++count;
    }
    else if (result != HAL_BUSY)
    {
      return result;
    }
    HAL_Delay(1U);
  }
  gyro_bias[0] = (float)sum[0] / (float)samples;
  gyro_bias[1] = (float)sum[1] / (float)samples;
  gyro_bias[2] = (float)sum[2] / (float)samples;
  return HAL_OK;
}

/* 将原始读数减去静止零偏，再按 ±1000 °/s 的灵敏度换算为角速度。 */
HAL_StatusTypeDef MPU6050_ReadGyro(MPU6050_Gyro *gyro)
{
  int32_t raw[3];
  HAL_StatusTypeDef result;

  if (initialized == 0U || gyro == NULL)
  {
    return HAL_ERROR;
  }
  result = MPU6050_ReadRaw(raw);
  if (result == HAL_OK)
  {
    gyro->x_dps = ((float)raw[0] - gyro_bias[0]) / MPU6050_GYRO_SCALE;
    gyro->y_dps = ((float)raw[1] - gyro_bias[1]) / MPU6050_GYRO_SCALE;
    gyro->z_dps = ((float)raw[2] - gyro_bias[2]) / MPU6050_GYRO_SCALE;
  }
  return result;
}
