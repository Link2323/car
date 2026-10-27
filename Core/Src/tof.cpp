#include "tof.h"
#include "../../ThirdParty/VL53L0X/VL53L0X.h"

/* 底层保存器件调优和校准流程；对其他 C 模块只暴露 tof.h 的简单接口。 */
/* 每路独立维护驱动实例、节拍及诊断信息，单路故障不会停掉其他路。 */
struct TOF_Context
{
  VL53L0X device;
  TOF_Data latest;
  uint8_t initialized;
  uint32_t last_poll_tick;
  uint32_t last_result_tick;
  HAL_StatusTypeDef status;
};
static TOF_Context sensors[TOF_SENSOR_COUNT];
static GPIO_TypeDef * const shut_ports[TOF_SENSOR_COUNT] = {
  TOF_F_XSHUT_GPIO_Port, TOF_L_XSHUT_GPIO_Port, TOF_R_XSHUT_GPIO_Port
};
static const uint16_t shut_pins[TOF_SENSOR_COUNT] = {
  TOF_F_XSHUT_Pin, TOF_L_XSHUT_Pin, TOF_R_XSHUT_Pin
};
static const uint8_t addresses[TOF_SENSOR_COUNT] = {
  TOF_FRONT_ADDRESS, TOF_LEFT_ADDRESS, TOF_RIGHT_ADDRESS
};
static const int32_t offsets[TOF_SENSOR_COUNT] = {
  TOF_FRONT_OFFSET_MM, TOF_LEFT_OFFSET_MM, TOF_RIGHT_OFFSET_MM
};

/* 使故障路数据失效并关闭该模块，防止部分初始化器件占用默认地址。 */
static HAL_StatusTypeDef TOF_Fail(TOF_Sensor sensor, TOF_State state, HAL_StatusTypeDef status)
{
  TOF_Context &ctx = sensors[sensor];
  ctx.initialized = 0U;
  ctx.latest.valid = 0U;
  ctx.latest.state = state;
  ctx.status = status;
  HAL_GPIO_WritePin(shut_ports[sensor], shut_pins[sensor], GPIO_PIN_RESET);
  return status;
}

/* 释放单路 XSHUT，用默认地址完成初始化后立即迁移至独立地址。 */
static HAL_StatusTypeDef TOF_InitSensor(TOF_Sensor sensor)
{
  TOF_Context &ctx = sensors[sensor];
  ctx.initialized = 0U;
  ctx.latest.distance_mm = 0U;
  ctx.latest.raw_distance_mm = 0U;
  ctx.latest.valid = 0U;
  ctx.latest.raw_status = 255U;
  ctx.latest.state = TOF_STATE_INIT_ERROR;
  ctx.latest.timestamp_ms = 0U;
  /* GPIO 为开漏；SET 表示释放 XSHUT，由模块自带上拉拉高。 */
  HAL_GPIO_WritePin(shut_ports[sensor], shut_pins[sensor], GPIO_PIN_SET);
  HAL_Delay(5U);
  ctx.device = VL53L0X();
  ctx.device.setTimeout(100U);
  if (!ctx.device.init())
  {
    /* 优先报告底层通信错误，区别于型号不符或参考校准失败。 */
    return TOF_Fail(sensor, ctx.device.bus_error == HAL_OK ? TOF_STATE_INIT_ERROR :
                   TOF_STATE_I2C_ERROR,
                   ctx.device.bus_error == HAL_OK ? HAL_ERROR : ctx.device.bus_error);
  }
  ctx.device.setAddress(addresses[sensor]);
  /* 按 33 ms 时间预算测量，每 50 ms 更新一次，主循环只取已完成的结果。 */
  if (!ctx.device.setMeasurementTimingBudget(33000U) || ctx.device.bus_error != HAL_OK)
  {
    return TOF_Fail(sensor, ctx.device.bus_error == HAL_OK ? TOF_STATE_INIT_ERROR :
                   TOF_STATE_I2C_ERROR,
                   ctx.device.bus_error == HAL_OK ? HAL_ERROR : ctx.device.bus_error);
  }
  ctx.device.startContinuous(50U);
  if (ctx.device.bus_error != HAL_OK)
  {
    return TOF_Fail(sensor, TOF_STATE_I2C_ERROR, ctx.device.bus_error);
  }
  ctx.last_poll_tick = HAL_GetTick();
  ctx.last_result_tick = ctx.last_poll_tick;
  ctx.latest.state = TOF_STATE_WAITING;
  ctx.initialized = 1U;
  return HAL_OK;
}

/* 轮询完成位，整帧读取状态与距离；不调用底层等待测量完成的阻塞接口。 */
static HAL_StatusTypeDef TOF_UpdateSensor(TOF_Sensor sensor)
{
  TOF_Context &ctx = sensors[sensor];
  uint32_t now = HAL_GetTick();
  uint8_t ready;
  uint8_t result[12];
  int32_t corrected_mm;
  if (ctx.initialized == 0U)
  {
    return ctx.status != HAL_OK ? ctx.status : HAL_ERROR;
  }
  /* 无符号差值兼容系统节拍回绕，也限制共享 I2C 总线的查询频率。 */
  if ((uint32_t)(now - ctx.last_poll_tick) < TOF_POLL_PERIOD_MS) { return HAL_BUSY; }
  ctx.last_poll_tick = now;
  ready = ctx.device.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (ctx.device.bus_error != HAL_OK)
  {
    return TOF_Fail(sensor, TOF_STATE_I2C_ERROR, ctx.device.bus_error);
  }
  if ((ready & 0x07U) == 0U)
  {
    /* 没有新结果时继续其他任务；超过期限则关闭并等待上层重新初始化。 */
    if ((uint32_t)(HAL_GetTick() - ctx.last_result_tick) >= TOF_SAMPLE_TIMEOUT_MS)
    {
      return TOF_Fail(sensor, TOF_STATE_TIMEOUT, HAL_TIMEOUT);
    }
    return HAL_BUSY;
  }
  ctx.device.readMulti(VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result));
  ctx.device.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01U);
  if (ctx.device.bus_error != HAL_OK)
  {
    return TOF_Fail(sensor, TOF_STATE_I2C_ERROR, ctx.device.bus_error);
  }
  ctx.latest.raw_status = (result[0] & 0x78U) >> 3;
  ctx.latest.raw_distance_mm = ((uint16_t)result[10] << 8) | result[11];
  /* 使用有符号中间值，避免近距离减去偏移后绕回很大的正数。
     保留原始值，补偿不能把坏回波或超量程值变成有效结果。 */
  corrected_mm = (int32_t)ctx.latest.raw_distance_mm - offsets[sensor];
  ctx.latest.distance_mm = corrected_mm <= 0 ? 0U :
                       (corrected_mm > 65535 ? 65535U : (uint16_t)corrected_mm);
  ctx.last_result_tick = HAL_GetTick();
  ctx.latest.timestamp_ms = ctx.last_result_tick;
  /* 11 是器件内部的有效测量码，并非 ST API 映射后的 RangeStatus=0。
     无目标、信号不足及 8190 等哨兵值均不能当作正常距离使用。 */
  ctx.latest.valid = (ctx.latest.raw_status == 11U && ctx.latest.raw_distance_mm > 0U &&
                  ctx.latest.raw_distance_mm <= 2000U && ctx.latest.distance_mm > 0U &&
                  ctx.latest.distance_mm <= 2000U) ? 1U : 0U;
  ctx.latest.state = ctx.latest.valid != 0U ? TOF_STATE_OK : TOF_STATE_RANGE_INVALID;
  return HAL_OK;
}

/* 提供值拷贝并再次检查新鲜度，避免调用者暂停采集后继续使用旧距离。 */
HAL_StatusTypeDef TOF_GetSensorData(TOF_Sensor sensor, TOF_Data *data)
{
  if ((unsigned int)sensor >= TOF_SENSOR_COUNT || data == NULL) { return HAL_ERROR; }
  TOF_Context &ctx = sensors[sensor];
  *data = ctx.latest;
  /* 静态零初始化时尚未调用 TOF_Init，不能将 state=0 视作已就绪。 */
  if (ctx.initialized == 0U && data->state == TOF_STATE_OK)
  {
    data->state = TOF_STATE_INIT_ERROR;
    data->raw_status = 255U;
  }
  if (data->valid != 0U &&
      (uint32_t)(HAL_GetTick() - data->timestamp_ms) >= TOF_SAMPLE_TIMEOUT_MS)
  {
    data->valid = 0U;
    data->state = TOF_STATE_TIMEOUT;
  }
  return HAL_OK;
}

/* 全部拉低后逐路启动，保证总线上最多只有一个器件使用默认地址。 */
HAL_StatusTypeDef TOF_Init(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  unsigned int i;
  /* 重复初始化同样先复位全部器件，清除上次留下的地址。 */
  for (i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    HAL_GPIO_WritePin(shut_ports[i], shut_pins[i], GPIO_PIN_RESET);
  }
  HAL_Delay(2U);
  /* 单路失败后仍尝试其他路，首个错误作为整体初始化结果。 */
  for (i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    sensors[i].status = TOF_InitSensor((TOF_Sensor)i);
    if (result == HAL_OK && sensors[i].status != HAL_OK)
    {
      result = sensors[i].status;
    }
  }
  /* 初始化三路可能耗时较长，从全部完成时开始计算首次结果超时。 */
  for (i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    sensors[i].last_result_tick = HAL_GetTick();
    sensors[i].last_poll_tick = HAL_GetTick();
  }
  return result;
}

/* 每轮服务全部模块，任何一路出错都不提前退出其他路的处理。 */
HAL_StatusTypeDef TOF_Update(void)
{
  HAL_StatusTypeDef result = HAL_BUSY;
  unsigned int i;
  /* HAL_BUSY 仅表示尚无新结果，不覆盖该路最后的实际通信状态。 */
  for (i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = TOF_UpdateSensor((TOF_Sensor)i);
    if (status != HAL_BUSY)
    {
      /* 已关闭的故障路保留最初的 HAL_TIMEOUT 等错误原因。 */
      if (sensors[i].initialized != 0U || sensors[i].status == HAL_OK)
      {
        sensors[i].status = status;
      }
      if (result == HAL_BUSY || (result == HAL_OK && status != HAL_OK))
      {
        result = status;
      }
    }
  }
  return result;
}

/* 兼容前方单路调用者，新增控制逻辑可直接使用按方向读取接口。 */
HAL_StatusTypeDef TOF_GetData(TOF_Data *data)
{
  return TOF_GetSensorData(TOF_FRONT, data);
}

/* 读取单路调试状态；首次初始化前统一返回错误。 */
HAL_StatusTypeDef TOF_GetSensorStatus(TOF_Sensor sensor)
{
  if ((unsigned int)sensor >= TOF_SENSOR_COUNT) { return HAL_ERROR; }
  if (sensors[sensor].latest.state == TOF_STATE_OK &&
      sensors[sensor].initialized == 0U) { return HAL_ERROR; }
  return sensors[sensor].status;
}
