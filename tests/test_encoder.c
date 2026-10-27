#include "encoder.h"
#include "tim.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
static TIM_TypeDef left_timer;
static TIM_TypeDef right_timer;
static uint32_t ticks;
static unsigned int start_calls;
static unsigned int stop_calls;
static unsigned int fail_at;

/* 返回模拟的毫秒节拍，验证延迟采样与计时回绕。 */
uint32_t HAL_GetTick(void)
{
  return ticks;
}

/* 校验编码器同时启动 A/B 通道，并可注入某一路启动失败。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  assert(timer == &htim3 || timer == &htim4);
  assert(channel == TIM_CHANNEL_ALL);
  ++start_calls;
  return (start_calls == fail_at) ? HAL_ERROR : HAL_OK;
}

/* 第二路启动失败时必须撤销第一路，避免半初始化。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  assert(timer == &htim3 && channel == TIM_CHANNEL_ALL);
  ++stop_calls;
  return HAL_OK;
}

/* 检查单精度计数率的舍入误差。 */
static void CheckNear(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.01f);
}

/* 覆盖双向回绕、实际采样间隔、软件清零、半范围歧义及启动失败。 */
int main(int argc, char **argv)
{
  Encoder_Data data = {0};
  Encoder_Data repeated;
  (void)argv;
  htim3.Instance = &left_timer;
  htim4.Instance = &right_timer;
  htim3.Init.Period = 65535U;
  htim4.Init.Period = 65535U;
  left_timer.CNT = 65530U;
  right_timer.CNT = 5U;
  assert(Encoder_Update() == HAL_ERROR);
  assert(Encoder_Reset() == HAL_ERROR);
  assert(Encoder_GetData(&data) == HAL_ERROR);

  /* 失败分支使用独立进程，避免成功初始化状态影响注入结果。 */
  if (argc > 1 && strcmp(argv[1], "forward") == 0)
  {
    /* 回放 COM7 实测：两轮均向前时，左硬件减计数、右硬件加计数。
       10 ms 内 -13/+12 对应修正前的 -1300/+1200 counts/s。 */
    assert(Encoder_Init() == HAL_OK);
    ticks = 10U;
    left_timer.CNT -= 13U;
    right_timer.CNT += 12U;
    assert(Encoder_Update() == HAL_OK);
    assert(Encoder_GetData(&data) == HAL_OK);
    assert(data.left_delta == 13 && data.right_delta == 12);
    CheckNear(data.left_counts_per_s, 1300.0f);
    CheckNear(data.right_counts_per_s, 1200.0f);
    puts("Measured forward encoder direction regression passed");
    return 0;
  }

  if (argc > 1)
  {
    fail_at = 2U;
    assert(Encoder_Init() == HAL_ERROR);
    assert(stop_calls == 1U);
    assert(Encoder_Update() == HAL_ERROR);
    puts("Encoder start rollback test passed");
    return 0;
  }

  htim3.Init.Period = 1000U;
  assert(Encoder_Init() == HAL_ERROR && start_calls == 0U);
  htim3.Init.Period = 65535U;
  assert(Encoder_Init() == HAL_OK && start_calls == 2U);
  assert(Encoder_Init() == HAL_BUSY);
  assert(Encoder_GetData(NULL) == HAL_ERROR);
  assert(Encoder_GetData(&data) == HAL_OK);
  assert(data.left_total == 0 && data.right_total == 0);
  assert(data.sample_valid == 0U && data.total_valid == 1U);
  ticks = 9U;
  left_timer.CNT = 4U;
  right_timer.CNT = 65530U;
  assert(Encoder_Update() == HAL_BUSY);
  ticks = 10U;
  assert(Encoder_Update() == HAL_OK);
  assert(Encoder_GetData(&data) == HAL_OK);
  assert(data.left_delta == 10 * ENCODER_LEFT_SIGN);
  assert(data.right_delta == -11 * ENCODER_RIGHT_SIGN);
  assert(data.left_total == data.left_delta && data.right_total == data.right_delta);
  CheckNear(data.left_counts_per_s, 1000.0f * ENCODER_LEFT_SIGN);
  CheckNear(data.right_counts_per_s, -1100.0f * ENCODER_RIGHT_SIGN);
  assert(data.interval_ms == 10U && data.sample_valid == 1U);

  /* 延迟到 40 ms 后才采样时，速度除以实际 40 ms，不能仍按 10 ms 算。 */
  ticks = 50U;
  left_timer.CNT += 40U;
  right_timer.CNT -= 20U;
  assert(Encoder_Update() == HAL_OK);
  assert(Encoder_GetData(&data) == HAL_OK);
  CheckNear(data.left_counts_per_s, 1000.0f * ENCODER_LEFT_SIGN);
  CheckNear(data.right_counts_per_s, -500.0f * ENCODER_RIGHT_SIGN);
  assert(Encoder_GetData(&repeated) == HAL_OK);
  assert(repeated.left_total == data.left_total);
  assert(Encoder_Update() == HAL_BUSY);

  assert(Encoder_Reset() == HAL_OK);
  assert(left_timer.CNT == 44U && right_timer.CNT == 65510U);
  assert(Encoder_GetData(&data) == HAL_OK && data.left_total == 0);
  ticks = 60U;
  left_timer.CNT = (uint16_t)(left_timer.CNT + 32768U);
  assert(Encoder_Update() == HAL_ERROR);
  assert(Encoder_GetData(&data) == HAL_OK);
  assert(data.sample_valid == 0U && data.total_valid == 0U && data.left_total == 0);
  ticks = 70U;
  left_timer.CNT += 2U;
  assert(Encoder_Update() == HAL_OK);
  assert(Encoder_GetData(&data) == HAL_OK);
  assert(data.sample_valid == 1U && data.total_valid == 0U);
  assert(data.left_delta == 2 * ENCODER_LEFT_SIGN);

  /* 两端计数和毫秒节拍同时回绕时，仍应得到正确方向和 10 ms 间隔。 */
  ticks = 0xFFFFFFFAU;
  left_timer.CNT = 0U;
  right_timer.CNT = 65535U;
  assert(Encoder_Reset() == HAL_OK);
  left_timer.CNT = 65535U;
  right_timer.CNT = 0U;
  ticks = 4U;
  assert(Encoder_Update() == HAL_OK);
  assert(Encoder_GetData(&data) == HAL_OK);
  assert(data.interval_ms == 10U && data.total_valid == 1U);
  assert(data.left_delta == -ENCODER_LEFT_SIGN);
  assert(data.right_delta == ENCODER_RIGHT_SIGN);
  ticks = 14U;
  assert(Encoder_Update() == HAL_OK);
  assert(Encoder_GetData(&data) == HAL_OK);
  CheckNear(data.left_counts_per_s, 0.0f);
  CheckNear(data.right_counts_per_s, 0.0f);
  puts("Encoder sampling tests passed");
  return 0;
}
