#include "tof.h"
#include "../../ThirdParty/VL53L0X/VL53L0X.h"

/* 底层保存器件调优和校准流程；对其他 C 模块只暴露 tof.h 的简单接口。 */
static VL53L0X front;
static TOF_Data latest = {0U, 0U, 255U, TOF_STATE_INIT_ERROR, 0U, 0U};
static uint8_t initialized;
static uint32_t last_poll_tick;
static uint32_t last_result_tick;

/* 将失败变为明确的无效状态，并关闭前方模块，防止使用部分初始化的器件。 */
static HAL_StatusTypeDef TOF_Fail(TOF_State state, HAL_StatusTypeDef status)
{
  initialized = 0U;
  latest.valid = 0U;
  latest.state = state;
  HAL_GPIO_WritePin(TOF_F_XSHUT_GPIO_Port, TOF_F_XSHUT_Pin, GPIO_PIN_RESET);
  return status;
}

/* 复位并只启动前方器件，先用默认地址初始化，再迁移至规划地址 0x31。 */
HAL_StatusTypeDef TOF_Init(void)
{
  initialized = 0U;
  latest.distance_mm = 0U;
  latest.raw_distance_mm = 0U;
  latest.valid = 0U;
  latest.raw_status = 255U;
  latest.state = TOF_STATE_INIT_ERROR;
  latest.timestamp_ms = 0U;
  HAL_GPIO_WritePin(TOF_L_XSHUT_GPIO_Port, TOF_L_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF_R_XSHUT_GPIO_Port, TOF_R_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF_F_XSHUT_GPIO_Port, TOF_F_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(2U);
  /* GPIO 为开漏；SET 表示释放 XSHUT，由模块自带上拉拉高。 */
  HAL_GPIO_WritePin(TOF_F_XSHUT_GPIO_Port, TOF_F_XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);
  front = VL53L0X();
  front.setTimeout(100U);
  if (!front.init())
  {
    /* 优先报告底层通信错误，区别于型号不符或参考校准失败。 */
    return TOF_Fail(front.bus_error == HAL_OK ? TOF_STATE_INIT_ERROR :
                   TOF_STATE_I2C_ERROR,
                   front.bus_error == HAL_OK ? HAL_ERROR : front.bus_error);
  }
  front.setAddress(TOF_FRONT_ADDRESS);
  /* 按 33 ms 时间预算测量，每 50 ms 更新一次，主循环只取已完成的结果。 */
  if (!front.setMeasurementTimingBudget(33000U) || front.bus_error != HAL_OK)
  {
    return TOF_Fail(front.bus_error == HAL_OK ? TOF_STATE_INIT_ERROR :
                   TOF_STATE_I2C_ERROR,
                   front.bus_error == HAL_OK ? HAL_ERROR : front.bus_error);
  }
  front.startContinuous(50U);
  if (front.bus_error != HAL_OK)
  {
    return TOF_Fail(TOF_STATE_I2C_ERROR, front.bus_error);
  }
  last_poll_tick = HAL_GetTick();
  last_result_tick = last_poll_tick;
  latest.state = TOF_STATE_WAITING;
  initialized = 1U;
  return HAL_OK;
}

/* 轮询完成位，整帧读取状态与距离；不调用底层等待测量完成的阻塞接口。 */
HAL_StatusTypeDef TOF_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t ready;
  uint8_t result[12];
  int32_t corrected_mm;
  if (initialized == 0U) { return HAL_ERROR; }
  /* 无符号差值兼容系统节拍回绕，也限制共享 I2C 总线的查询频率。 */
  if ((uint32_t)(now - last_poll_tick) < TOF_POLL_PERIOD_MS) { return HAL_BUSY; }
  last_poll_tick = now;
  ready = front.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (front.bus_error != HAL_OK)
  {
    return TOF_Fail(TOF_STATE_I2C_ERROR, front.bus_error);
  }
  if ((ready & 0x07U) == 0U)
  {
    /* 没有新结果时继续其他任务；超过期限则关闭并等待上层重新初始化。 */
    if ((uint32_t)(HAL_GetTick() - last_result_tick) >= TOF_SAMPLE_TIMEOUT_MS)
    {
      return TOF_Fail(TOF_STATE_TIMEOUT, HAL_TIMEOUT);
    }
    return HAL_BUSY;
  }
  front.readMulti(VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result));
  front.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01U);
  if (front.bus_error != HAL_OK)
  {
    return TOF_Fail(TOF_STATE_I2C_ERROR, front.bus_error);
  }
  latest.raw_status = (result[0] & 0x78U) >> 3;
  latest.raw_distance_mm = ((uint16_t)result[10] << 8) | result[11];
  /* 使用有符号中间值，避免近距离减去偏移后绕回很大的正数。
     保留原始值，补偿不能把坏回波或超量程值变成有效结果。 */
  corrected_mm = (int32_t)latest.raw_distance_mm - TOF_FRONT_OFFSET_MM;
  latest.distance_mm = corrected_mm <= 0 ? 0U :
                       (corrected_mm > 65535 ? 65535U : (uint16_t)corrected_mm);
  last_result_tick = HAL_GetTick();
  latest.timestamp_ms = last_result_tick;
  /* 11 是器件内部的有效测量码，并非 ST API 映射后的 RangeStatus=0。
     无目标、信号不足及 8190 等哨兵值均不能当作正常距离使用。 */
  latest.valid = (latest.raw_status == 11U && latest.raw_distance_mm > 0U &&
                  latest.raw_distance_mm <= 2000U && latest.distance_mm > 0U &&
                  latest.distance_mm <= 2000U) ? 1U : 0U;
  latest.state = latest.valid != 0U ? TOF_STATE_OK : TOF_STATE_RANGE_INVALID;
  return HAL_OK;
}

/* 提供值拷贝并再次检查新鲜度，避免调用者暂停采集后继续使用旧距离。 */
HAL_StatusTypeDef TOF_GetData(TOF_Data *data)
{
  if (data == NULL) { return HAL_ERROR; }
  *data = latest;
  if (data->valid != 0U &&
      (uint32_t)(HAL_GetTick() - data->timestamp_ms) >= TOF_SAMPLE_TIMEOUT_MS)
  {
    data->valid = 0U;
    data->state = TOF_STATE_TIMEOUT;
  }
  return HAL_OK;
}
