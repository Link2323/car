#include "encoder.h"
#include "tim.h"

#if (ENCODER_LEFT_SIGN != 1 && ENCODER_LEFT_SIGN != -1) || \
    (ENCODER_RIGHT_SIGN != 1 && ENCODER_RIGHT_SIGN != -1)
#error "Encoder direction must be +1 or -1"
#endif

static uint8_t initialized;
static uint16_t previous_left;
static uint16_t previous_right;
static uint32_t previous_tick;
static Encoder_Data snapshot;

/* 对 16 位无符号差值做补码解码，处理 65535→0 和 0→65535 的双向回绕。
   显式减去 65536，避免依赖 uint16_t 转 int16_t 的实现相关结果。 */
static int32_t Encoder_DecodeDelta(uint16_t difference)
{
  return (difference >= 32768U) ? (int32_t)difference - 65536 : (int32_t)difference;
}

/* 启动后只建立计数基线，不把初始化前已经存在的 CNT 误计为位移。 */
HAL_StatusTypeDef Encoder_Init(void)
{
  if (initialized != 0U)
  {
    return HAL_BUSY;
  }
  /* 模运算以 65536 为周期，PSC 必须为零，避免计数分频造成单位不一致。 */
  if (htim3.Init.Period != 65535U || htim4.Init.Period != 65535U ||
      htim3.Init.Prescaler != 0U || htim4.Init.Prescaler != 0U)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    /* 第二路启动失败时撤销第一路，避免留下半初始化状态。 */
    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    return HAL_ERROR;
  }
  initialized = 1U;
  return Encoder_Reset();
}

/* 读取硬件当前值作为新基线；不写 CNT，避免运行中读后清零丢失边沿。 */
HAL_StatusTypeDef Encoder_Reset(void)
{
  Encoder_Data cleared = {0};
  if (initialized == 0U)
  {
    return HAL_ERROR;
  }
  previous_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  previous_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  previous_tick = HAL_GetTick();
  cleared.timestamp_ms = previous_tick;
  cleared.total_valid = 1U;
  snapshot = cleared;
  return HAL_OK;
}

/* 按实际经过时间计算计数率，主循环迟到时不假设固定经过了 10 ms。 */
HAL_StatusTypeDef Encoder_Update(void)
{
  uint32_t now;
  uint32_t interval;
  uint16_t current_left;
  uint16_t current_right;
  uint16_t left_difference;
  uint16_t right_difference;

  if (initialized == 0U)
  {
    return HAL_ERROR;
  }
  now = HAL_GetTick();
  interval = (uint32_t)(now - previous_tick);
  /* 保护除数并限制采样频率；忙时保持上一份快照，调用者可重复读取。 */
  if (interval < ENCODER_SAMPLE_PERIOD_MS)
  {
    return HAL_BUSY;
  }
  current_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  current_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  left_difference = (uint16_t)(current_left - previous_left);
  right_difference = (uint16_t)(current_right - previous_right);
  previous_left = current_left;
  previous_right = current_right;
  previous_tick = now;
  snapshot.interval_ms = interval;
  snapshot.timestamp_ms = now;

  /* 恰好相差半个计数范围时无法区分正反方向，保留累计值但标记本次无效。
     更大的真实位移可能混叠成较小差值，软件无法仅靠两个 CNT 读数识别。 */
  if (left_difference == 32768U || right_difference == 32768U)
  {
    snapshot.left_delta = 0;
    snapshot.right_delta = 0;
    snapshot.left_counts_per_s = 0.0f;
    snapshot.right_counts_per_s = 0.0f;
    snapshot.sample_valid = 0U;
    snapshot.total_valid = 0U;
    return HAL_ERROR;
  }
  snapshot.left_delta = Encoder_DecodeDelta(left_difference) * ENCODER_LEFT_SIGN;
  snapshot.right_delta = Encoder_DecodeDelta(right_difference) * ENCODER_RIGHT_SIGN;
  snapshot.left_total += snapshot.left_delta;
  snapshot.right_total += snapshot.right_delta;
  snapshot.left_counts_per_s = (float)snapshot.left_delta * 1000.0f / (float)interval;
  snapshot.right_counts_per_s = (float)snapshot.right_delta * 1000.0f / (float)interval;
  snapshot.sample_valid = 1U;
  return HAL_OK;
}

/* 在主循环复制一致的双轮快照，不进行硬件清零或再次累计。 */
HAL_StatusTypeDef Encoder_GetData(Encoder_Data *data)
{
  if (initialized == 0U || data == NULL)
  {
    return HAL_ERROR;
  }
  *data = snapshot;
  return HAL_OK;
}
