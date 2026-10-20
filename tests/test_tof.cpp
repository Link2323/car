/* 模拟器件分页寄存器与 XSHUT，覆盖实际移植库和上层驱动，不依赖实物。 */
#include "tof.h"
#include "i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
static uint8_t registers[8][256];
static uint8_t page;
static uint16_t address;
static uint32_t ticks;
static unsigned int transactions;
static uint8_t powered;
static uint8_t fail_read;
static uint8_t fail_write;
static uint8_t wrong_model;
static uint8_t hold_spad;
static uint8_t ready;

/* 返回模拟时间；总线访问和延时会推进此计数。 */
uint32_t HAL_GetTick(void) { return ticks; }

/* 只推进虚拟时间，测试无需真实等待。 */
void HAL_Delay(uint32_t delay) { ticks += delay; }

/* 检查左右模块始终关闭，前方释放 XSHUT 时模拟器件恢复出厂地址。 */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  if ((port == TOF_L_XSHUT_GPIO_Port && pin == TOF_L_XSHUT_Pin) ||
      (port == TOF_R_XSHUT_GPIO_Port && pin == TOF_R_XSHUT_Pin))
  {
    assert(state == GPIO_PIN_RESET);
    return;
  }
  assert(port == TOF_F_XSHUT_GPIO_Port && pin == TOF_F_XSHUT_Pin);
  powered = state == GPIO_PIN_SET;
  if (powered != 0U)
  {
    memset(registers, 0, sizeof(registers));
    page = 0U;
    address = 0x29U << 1;
    ready = 1U;
    registers[0][0xC0] = wrong_model != 0U ? 0U : 0xEEU;
    registers[7][0x92] = 0x85U;
    memset(&registers[0][0xB0], 0xFF, 6);
    registers[0][0x50] = 6U;
    registers[0][0x70] = 4U;
    registers[0][0x51] = 0U;
    registers[0][0x52] = 0x96U;
    registers[0][0x71] = 1U;
    registers[0][0x72] = 0xFEU;
  }
}

/* 模拟连续寄存器读取、参考 SPAD 完成位及测量就绪位，可注入通信失败。 */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == &hi2c1 && reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 10U);
  ++transactions;
  ++ticks;
  if (fail_read != 0U || powered == 0U || addr != address) { return HAL_ERROR; }
  assert(reg + length <= 256U);
  memcpy(data, &registers[page][reg], length);
  if (page == 7U && reg == 0x83U) { data[0] = hold_spad != 0U ? 0U : 1U; }
  if (page == 0U && reg == 0x13U) { data[0] = ready != 0U ? 7U : 0U; }
  return HAL_OK;
}

/* 模拟寄存器分页和改地址，并在清除中断后撤销本帧就绪位。 */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr,
  uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == &hi2c1 && reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 10U);
  ++transactions;
  ++ticks;
  if (fail_write != 0U || powered == 0U || addr != address) { return HAL_TIMEOUT; }
  assert(reg + length <= 256U);
  memcpy(&registers[page][reg], data, length);
  if (page == 0U && reg == 0x8AU) { address = (uint16_t)data[0] << 1; }
  if (page == 0U && reg == 0x0BU) { ready = 0U; }
  /* 校准启动后立即产生结果；正常测量由测试显式注入回波。 */
  if (page == 0U && reg == 0U && (data[0] & 1U) != 0U) { ready = 1U; }
  if (reg == 0xFFU) { page = data[0]; assert(page < 8U); }
  return HAL_OK;
}

/* 重新初始化并检查地址迁移及首帧前的等待状态。 */
static void InitGood(void)
{
  TOF_Data data;
  fail_read = fail_write = wrong_model = hold_spad = 0U;
  assert(TOF_Init() == HAL_OK);
  assert(address == (TOF_FRONT_ADDRESS << 1));
  assert(TOF_GetData(&data) == HAL_OK);
  assert(data.valid == 0U && data.state == TOF_STATE_WAITING);
  ready = 0U;
}

/* 注入整帧回波，推进到查询周期后检查返回快照。 */
static TOF_Data Sample(uint16_t mm, uint8_t raw_status)
{
  TOF_Data data;
  registers[0][0x14] = (uint8_t)(raw_status << 3);
  registers[0][0x1E] = (uint8_t)(mm >> 8);
  registers[0][0x1F] = (uint8_t)mm;
  ready = 1U;
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_OK);
  assert(TOF_GetData(&data) == HAL_OK);
  assert(ready == 0U);
  return data;
}

/* 验证有效/无效回波、节流、超时、断线、故障恢复和节拍回绕。 */
int main(void)
{
  TOF_Data data;
  unsigned int before;
  uint32_t start;
  assert(TOF_GetData(NULL) == HAL_ERROR);
  assert(TOF_Update() == HAL_ERROR);
  InitGood();
  before = transactions;
  assert(TOF_Update() == HAL_BUSY && transactions == before);
  /* 用用户提供的实测数据验证固定偏移校正，先排除只改变显示文字的假修复。 */
  data = Sample(130U, 11U);
  assert(data.distance_mm == 98U && data.raw_distance_mm == 130U && data.valid == 1U);
  data = Sample(232U, 11U);
  assert(data.distance_mm == 200U && data.valid == 1U);
  data = Sample(260U, 11U);
  assert(data.distance_mm == 228U && data.valid == 1U);
  /* 低于校正量时不得无符号下溢，也不把截断到零的值视为可信测量。 */
  data = Sample(31U, 11U);
  assert(data.distance_mm == 0U && data.valid == 0U);
  data = Sample(32U, 11U);
  assert(data.distance_mm == 0U && data.valid == 0U);
  data = Sample(1234U, 11U);
  assert(data.distance_mm == 1202U && data.valid == 1U && data.state == TOF_STATE_OK);
  data = Sample(8190U, 4U);
  assert(data.valid == 0U && data.state == TOF_STATE_RANGE_INVALID && data.raw_status == 4U);
  data = Sample(2001U, 11U);
  assert(data.valid == 0U);
  data = Sample(0U, 11U);
  assert(data.valid == 0U);
  data = Sample(2000U, 11U);
  assert(data.valid == 1U);
  ticks += TOF_SAMPLE_TIMEOUT_MS;
  assert(TOF_GetData(&data) == HAL_OK && data.valid == 0U && data.state == TOF_STATE_TIMEOUT);
  assert(TOF_Update() == HAL_TIMEOUT && powered == 0U);
  InitGood();
  fail_read = 1U;
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_ERROR);
  assert(TOF_GetData(&data) == HAL_OK && data.state == TOF_STATE_I2C_ERROR && data.valid == 0U);
  before = transactions;
  assert(TOF_Update() == HAL_ERROR && transactions == before);
  InitGood();
  fail_write = 1U;
  ready = 1U;
  ticks += TOF_POLL_PERIOD_MS;
  assert(TOF_Update() == HAL_TIMEOUT && powered == 0U);
  assert(TOF_GetData(&data) == HAL_OK && data.valid == 0U && data.state == TOF_STATE_I2C_ERROR);
  InitGood();
  wrong_model = 1U;
  assert(TOF_Init() == HAL_ERROR && powered == 0U);
  assert(TOF_GetData(&data) == HAL_OK && data.state == TOF_STATE_INIT_ERROR);
  wrong_model = 0U;
  hold_spad = 1U;
  start = ticks;
  assert(TOF_Init() == HAL_ERROR && powered == 0U);
  assert((uint32_t)(ticks - start) < 500U);
  hold_spad = 0U;
  fail_read = 1U;
  assert(TOF_Init() == HAL_ERROR && powered == 0U);
  InitGood();
  ticks = 0xFFFFFF00U;
  data = Sample(321U, 11U);
  assert(data.valid == 1U);
  ticks += 150U;
  assert(TOF_GetData(&data) == HAL_OK && data.valid == 1U);
  ticks += 100U;
  assert(TOF_GetData(&data) == HAL_OK && data.valid == 0U);
  puts("PASS: TOF init/address, range/status, offset calibration, polling, stale/timeout, I2C failures, recovery, tick wrap");
  return 0;
}
