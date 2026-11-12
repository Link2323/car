/* 模拟器件分页寄存器与 XSHUT，覆盖实际移植库和上层驱动，不依赖实物。 */
#include "tof.h"
#include "i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
struct SimDevice
{
  uint8_t registers[8][256];
  uint8_t page;
  uint16_t address;
  uint8_t powered;
  uint8_t fail_read;
  uint8_t fail_write;
  uint8_t wrong_model;
  uint8_t hold_spad;
  uint8_t ready;
};
static SimDevice devices[TOF_SENSOR_COUNT];
static uint32_t ticks;
static unsigned int transactions;
/* 本场景刻意不提供模块外部上拉，只有输入上拉才能释放 XSHUT。 */
static uint32_t pin_modes[TOF_SENSOR_COUNT];
static uint32_t pin_pulls[TOF_SENSOR_COUNT];
static uint8_t pin_odr[TOF_SENSOR_COUNT];

/* 按地址查找唯一响应者，直接检查是否发生同地址冲突。 */
static SimDevice *FindDevice(uint16_t addr)
{
  SimDevice *found = NULL;
  for (unsigned int i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    if (devices[i].powered != 0U && devices[i].address == addr)
    {
      assert(found == NULL);
      found = &devices[i];
    }
  }
  return found;
}

/* 返回模拟时间；总线访问和延时会推进此计数。 */
uint32_t HAL_GetTick(void) { return ticks; }

/* 只推进虚拟时间，测试无需真实等待。 */
void HAL_Delay(uint32_t delay) { ticks += delay; }

/* 将端口和引脚解析为方向，防止错误引脚或一次操作多路掩盖地址冲突。 */
static unsigned int PinIndex(GPIO_TypeDef *port, uint16_t pin)
{
  unsigned int index = TOF_SENSOR_COUNT;
  if (port == TOF_F_XSHUT_GPIO_Port && pin == TOF_F_XSHUT_Pin) { index = TOF_FRONT; }
  if (port == TOF_L_XSHUT_GPIO_Port && pin == TOF_L_XSHUT_Pin) { index = TOF_LEFT; }
  if (port == TOF_R_XSHUT_GPIO_Port && pin == TOF_R_XSHUT_Pin) { index = TOF_RIGHT; }
  assert(index < TOF_SENSOR_COUNT);
  return index;
}

/* 只在 XSHUT 真实上升沿恢复默认地址；开漏写高但缺少上拉时仍保持关断。 */
static void RefreshXshut(unsigned int index)
{
  SimDevice &d = devices[index];
  uint8_t high = pin_modes[index] == GPIO_MODE_INPUT &&
                 pin_pulls[index] == GPIO_PULLUP && pin_odr[index] != 0U;
  if (high == d.powered) { return; }
  if (high != 0U) { assert(FindDevice(0x29U << 1) == NULL); }
  d.powered = high;
  if (high != 0U)
  {
    memset(d.registers, 0, sizeof(d.registers));
    d.page = 0U;
    d.address = 0x29U << 1;
    d.ready = 1U;
    d.registers[0][0xC0] = d.wrong_model != 0U ? 0U : 0xEEU;
    d.registers[7][0x92] = 0x85U;
    memset(&d.registers[0][0xB0], 0xFF, 6);
    d.registers[0][0x50] = 6U;
    d.registers[0][0x70] = 4U;
    d.registers[0][0x51] = 0U;
    d.registers[0][0x52] = 0x96U;
    d.registers[0][0x71] = 1U;
    d.registers[0][0x72] = 0xFEU;
  }
}

/* ODR 只负责输出锁存，是否产生高电平由引脚模式与上拉共同决定。 */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  unsigned int index = PinIndex(port, pin);
  pin_odr[index] = state == GPIO_PIN_SET;
  RefreshXshut(index);
}

/* 模拟 F1 输入上拉会设置 ODR；关断必须使用开漏低电平，不允许推挽高。 */
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
  unsigned int index = PinIndex(port, (uint16_t)config->Pin);
  assert((config->Mode == GPIO_MODE_INPUT && config->Pull == GPIO_PULLUP) ||
         (config->Mode == GPIO_MODE_OUTPUT_OD && config->Pull == GPIO_NOPULL));
  pin_modes[index] = config->Mode;
  pin_pulls[index] = config->Pull;
  if (config->Mode == GPIO_MODE_INPUT) { pin_odr[index] = 1U; }
  else { assert(pin_odr[index] == 0U); }
  RefreshXshut(index);
}

/* 模拟连续寄存器读取、参考 SPAD 完成位及测量就绪位，可注入通信失败。 */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == &hi2c1 && reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 10U);
  ++transactions;
  ++ticks;
  SimDevice *selected = FindDevice(addr);
  if (selected == NULL || selected->fail_read != 0U) { return HAL_ERROR; }
  SimDevice &d = *selected;
  assert(reg + length <= 256U);
  memcpy(data, &d.registers[d.page][reg], length);
  if (d.page == 7U && reg == 0x83U) { data[0] = d.hold_spad != 0U ? 0U : 1U; }
  if (d.page == 0U && reg == 0x13U) { data[0] = d.ready != 0U ? 7U : 0U; }
  return HAL_OK;
}

/* 模拟寄存器分页和改地址，并在清除中断后撤销本帧就绪位。 */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == &hi2c1 && reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 10U);
  ++transactions;
  ++ticks;
  SimDevice *selected = FindDevice(addr);
  if (selected == NULL || selected->fail_write != 0U) { return HAL_TIMEOUT; }
  SimDevice &d = *selected;
  assert(reg + length <= 256U);
  memcpy(&d.registers[d.page][reg], data, length);
  if (d.page == 0U && reg == 0x8AU) { d.address = (uint16_t)data[0] << 1; }
  if (d.page == 0U && reg == 0x0BU) { d.ready = 0U; }
  /* 校准启动后立即产生结果；正常测量由测试显式注入回波。 */
  if (d.page == 0U && reg == 0U && (data[0] & 1U) != 0U) { d.ready = 1U; }
  if (reg == 0xFFU) { d.page = data[0]; assert(d.page < 8U); }
  return HAL_OK;
}

/* 逐路检查初始化后的独立地址、等待状态及通信状态。 */
static void InitGood(void)
{
  memset(devices, 0, sizeof(devices));
  assert(TOF_Init() == HAL_OK);
  const uint8_t addresses[] = {TOF_FRONT_ADDRESS, TOF_LEFT_ADDRESS, TOF_RIGHT_ADDRESS};
  for (unsigned int i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    TOF_Data data;
    assert(devices[i].address == (addresses[i] << 1));
    assert(pin_modes[i] == GPIO_MODE_INPUT && pin_pulls[i] == GPIO_PULLUP);
    assert(TOF_GetSensorData((TOF_Sensor)i, &data) == HAL_OK);
    assert(data.valid == 0U && data.state == TOF_STATE_WAITING);
    assert(TOF_GetSensorStatus((TOF_Sensor)i) == HAL_OK);
    devices[i].ready = 0U;
  }
}

/* 为指定方向注入整帧回波，验证三路寄存器不会串读。 */
static void Echo(TOF_Sensor sensor, uint16_t mm, uint8_t raw_status = 11U)
{
  SimDevice &d = devices[sensor];
  d.registers[0][0x14] = (uint8_t)(raw_status << 3);
  d.registers[0][0x1E] = (uint8_t)(mm >> 8);
  d.registers[0][0x1F] = (uint8_t)mm;
  d.ready = 1U;
}

/* 读取指定路快照，简化后续对有效性及故障隔离的断言。 */
static TOF_Data Data(TOF_Sensor sensor)
{
  TOF_Data data;
  assert(TOF_GetSensorData(sensor, &data) == HAL_OK);
  return data;
}

/* 验证三路改址、独立补偿、异常回波、局部故障、恢复及节拍回绕。 */
int main(void)
{
  TOF_Data data;
  assert(TOF_GetData(NULL) == HAL_ERROR);
  assert(TOF_GetSensorData(TOF_SENSOR_COUNT, &data) == HAL_ERROR);
  assert(TOF_GetSensorData((TOF_Sensor)-1, &data) == HAL_ERROR);
  assert(TOF_GetSensorStatus(TOF_FRONT) == HAL_ERROR);
  assert(TOF_Update() == HAL_ERROR);
  InitGood();
  unsigned int before = transactions;
  assert(TOF_Update() == HAL_BUSY && transactions == before);
  Echo(TOF_FRONT, 232U); Echo(TOF_LEFT, 300U); Echo(TOF_RIGHT, 400U);
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_OK);
  assert(Data(TOF_FRONT).distance_mm == 200U && Data(TOF_FRONT).valid == 1U);
  assert(Data(TOF_LEFT).distance_mm == 300U && Data(TOF_LEFT).valid == 1U);
  assert(Data(TOF_RIGHT).distance_mm == 400U && Data(TOF_RIGHT).valid == 1U);
  assert(TOF_GetData(&data) == HAL_OK && data.distance_mm == 200U);
  /* 所有方向均拒绝错误状态、哨兵值及原始超量程数据。 */
  const uint16_t invalid_mm[] = {0U, 2001U, 8190U};
  for (unsigned int k = 0U; k < 3U; ++k)
  {
    for (unsigned int i = 0U; i < TOF_SENSOR_COUNT; ++i)
    {
      Echo((TOF_Sensor)i, invalid_mm[k]);
    }
    ticks += TOF_POLL_PERIOD_MS;
    assert(TOF_Update() == HAL_OK);
    for (unsigned int i = 0U; i < TOF_SENSOR_COUNT; ++i)
    {
      assert(Data((TOF_Sensor)i).valid == 0U);
    }
  }
  Echo(TOF_FRONT, 31U); Echo(TOF_LEFT, 100U, 4U); Echo(TOF_RIGHT, 2000U);
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_OK);
  assert(Data(TOF_FRONT).distance_mm == 0U && Data(TOF_FRONT).valid == 0U);
  assert(Data(TOF_LEFT).state == TOF_STATE_RANGE_INVALID);
  assert(Data(TOF_RIGHT).valid == 1U);
  /* 前方断线后左右仍读到新帧；下一轮右侧清中断失败也不影响左侧。 */
  devices[TOF_FRONT].fail_read = 1U;
  Echo(TOF_LEFT, 333U); Echo(TOF_RIGHT, 444U);
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_ERROR);
  assert(devices[TOF_FRONT].powered == 0U);
  assert(Data(TOF_FRONT).state == TOF_STATE_I2C_ERROR);
  assert(Data(TOF_LEFT).distance_mm == 333U && Data(TOF_RIGHT).distance_mm == 444U);
  devices[TOF_RIGHT].fail_write = 1U;
  Echo(TOF_LEFT, 555U); Echo(TOF_RIGHT, 666U);
  ticks += TOF_POLL_PERIOD_MS;
  (void)TOF_Update();
  assert(Data(TOF_LEFT).distance_mm == 555U && Data(TOF_LEFT).valid == 1U);
  assert(TOF_GetSensorStatus(TOF_RIGHT) == HAL_TIMEOUT);
  ticks += TOF_POLL_PERIOD_MS;
  (void)TOF_Update();
  assert(TOF_GetSensorStatus(TOF_RIGHT) == HAL_TIMEOUT);
  /* 重初始化复位已有地址；左路身份错误时，后面的右路仍可完成初始化。 */
  InitGood();
  devices[TOF_LEFT].wrong_model = 1U;
  assert(TOF_Init() == HAL_ERROR);
  assert(devices[TOF_LEFT].powered == 0U);
  assert(Data(TOF_LEFT).state == TOF_STATE_INIT_ERROR);
  assert(TOF_GetSensorStatus(TOF_FRONT) == HAL_OK);
  assert(TOF_GetSensorStatus(TOF_RIGHT) == HAL_OK);
  /* SPAD 校准超时有界，前后方向不会被失败的默认地址器件占用。 */
  InitGood();
  devices[TOF_LEFT].hold_spad = 1U;
  uint32_t start = ticks;
  assert(TOF_Init() == HAL_ERROR);
  assert((uint32_t)(ticks - start) < 1500U);
  assert(devices[TOF_LEFT].powered == 0U && devices[TOF_RIGHT].powered == 1U);
  /* 仅左路无新帧超过期限，其余两路的新结果仍有效。 */
  InitGood();
  ticks += TOF_SAMPLE_TIMEOUT_MS;
  Echo(TOF_FRONT, 232U); Echo(TOF_RIGHT, 777U);
  assert(TOF_Update() == HAL_TIMEOUT);
  assert(Data(TOF_LEFT).state == TOF_STATE_TIMEOUT);
  assert(Data(TOF_FRONT).valid == 1U && Data(TOF_RIGHT).valid == 1U);
  ticks += TOF_SAMPLE_TIMEOUT_MS;
  assert(Data(TOF_FRONT).state == TOF_STATE_TIMEOUT && Data(TOF_FRONT).valid == 0U);
  /* 右路一次通信失败后，在禁止恢复时不访问总线；允许恢复后只复位右路。 */
  InitGood();
  devices[TOF_RIGHT].fail_read = 1U;
  ticks += TOF_POLL_PERIOD_MS;
  (void)TOF_Update();
  assert(Data(TOF_RIGHT).state == TOF_STATE_I2C_ERROR);
  devices[TOF_RIGHT].fail_read = 0U;
  ticks += TOF_RECOVERY_INTERVAL_MS;
  before = transactions;
  assert(TOF_Recover(0U) == HAL_BUSY && transactions == before);
  assert(TOF_Recover(1U) == HAL_OK);
  assert(devices[TOF_RIGHT].powered == 1U);
  assert(devices[TOF_FRONT].address == (TOF_FRONT_ADDRESS << 1));
  assert(devices[TOF_LEFT].address == (TOF_LEFT_ADDRESS << 1));
  Echo(TOF_FRONT, 232U); Echo(TOF_LEFT, 333U); Echo(TOF_RIGHT, 444U);
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_OK);
  assert(Data(TOF_RIGHT).valid == 1U && Data(TOF_RIGHT).distance_mm == 444U);
  /* 持续故障只能重试三次；次数用完后不再阻塞主循环，显式初始化可重置。 */
  devices[TOF_RIGHT].fail_read = 1U;
  ticks += TOF_POLL_PERIOD_MS;
  (void)TOF_Update();
  for (unsigned int attempt = 0U; attempt < TOF_RECOVERY_MAX_ATTEMPTS; ++attempt)
  {
    before = transactions;
    assert(TOF_Recover(1U) == HAL_BUSY && transactions == before);
    ticks += TOF_RECOVERY_INTERVAL_MS;
    assert(TOF_Recover(1U) == HAL_ERROR);
  }
  ticks += TOF_RECOVERY_INTERVAL_MS;
  before = transactions;
  assert(TOF_Recover(1U) == HAL_BUSY && transactions == before);
  /* 在节拍回绕之前开始初始化，跨回绕后仍能正确节流及判断新鲜度。 */
  ticks = 0xFFFFFE00U;
  InitGood();
  Echo(TOF_FRONT, 321U); Echo(TOF_LEFT, 322U); Echo(TOF_RIGHT, 323U);
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_OK);
  ticks += 150U;
  assert(Data(TOF_RIGHT).valid == 1U);
  ticks += 100U;
  assert(Data(TOF_RIGHT).valid == 0U);
  puts("PASS: three TOF addresses, independent data/offsets, invalid ranges, polling, fault isolation, timeout, recovery, tick wrap");
  return 0;
}
