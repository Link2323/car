/* 陀螺仪闭环转向主机测试：真实 heading/turn_control 链接，桩替速度环与节拍。
   验证目标换算、差速方向与限幅、到位确认、超时和陀螺失效保护。 */
#include "turn_control.h"
#include "speed_control.h"
#include "heading.h"
#include <assert.h>
#include <stdio.h>

/* ---- 测试桩：可控节拍 ---- */
static uint32_t ticks;
uint32_t HAL_GetTick(void) { return ticks; }

/* ---- 测试桩：速度环接口，记录调用并允许注入故障 ---- */
static uint8_t sc_enable_called;
static uint8_t sc_enable_result_ok = 1U;
static uint8_t sc_stop_called;
static uint32_t sc_stop_count;
static float sc_left_target;
static float sc_right_target;
static uint32_t sc_settarget_count;
static HAL_StatusTypeDef sc_settarget_result = HAL_OK;
static SpeedControl_State sc_state = SPEED_CONTROL_IDLE;

HAL_StatusTypeDef SpeedControl_Init(const SpeedControl_Config *config)
{
  (void)config;
  return HAL_OK;
}

HAL_StatusTypeDef SpeedControl_Enable(void)
{
  sc_enable_called = 1U;
  if (sc_enable_result_ok == 0U) { return HAL_ERROR; }
  sc_state = SPEED_CONTROL_RUNNING;
  return HAL_OK;
}

void SpeedControl_Stop(void)
{
  sc_stop_called = 1U;
  sc_stop_count++;
  sc_state = SPEED_CONTROL_IDLE;
}

HAL_StatusTypeDef SpeedControl_SetTarget(float left, float right)
{
  if (sc_settarget_result != HAL_OK) { return sc_settarget_result; }
  sc_left_target = left;
  sc_right_target = right;
  sc_settarget_count++;
  return HAL_OK;
}

HAL_StatusTypeDef SpeedControl_GetData(SpeedControl_Data *data)
{
  data->state = sc_state;
  return HAL_OK;
}

/* ---- 辅助：按毫秒推进并喂恒定 z 角速度样本，模拟真实旋转 ---- */
static void FeedGyro(uint32_t timestamp_ms, float z_dps)
{
  MPU6050_Gyro gyro = {0.0f, 0.0f, z_dps};
  Heading_Update(&gyro, timestamp_ms);
}

static void ResetStubs(void)
{
  ticks = 0U;
  sc_enable_called = 0U;
  sc_enable_result_ok = 1U;
  sc_stop_called = 0U;
  sc_stop_count = 0U;
  sc_left_target = 0.0f;
  sc_right_target = 0.0f;
  sc_settarget_count = 0U;
  sc_settarget_result = HAL_OK;
  sc_state = SPEED_CONTROL_IDLE;
}

int main(void)
{
  TurnControl_Data turn;
  const float tolerance = 2.5f;

  /* 未初始化时拒绝启动。 */
  assert(TurnControl_Start(90.0f) == HAL_ERROR);

  /* 初始化成功，重复初始化返回忙。 */
  assert(TurnControl_Init() == HAL_OK);
  assert(TurnControl_Init() == HAL_BUSY);

  /* 陀螺仪尚无有效样本时拒绝启动。 */
  assert(TurnControl_Start(90.0f) == HAL_ERROR);

  /* 建立积分基准：t=100 首份样本，z 角度从零开始。 */
  ResetStubs();
  Heading_Reset();
  ticks = 100U;
  FeedGyro(ticks, 0.0f);

  /* 非法角度：零、超范围。 */
  assert(TurnControl_Start(0.0f) == HAL_ERROR);
  assert(TurnControl_Start(361.0f) == HAL_ERROR);
  assert(TurnControl_Start(-361.0f) == HAL_ERROR);

  /* 左转 90 度：目标为基准 +90，速度环被使能。 */
  assert(TurnControl_Start(90.0f) == HAL_OK);
  assert(sc_enable_called == 1U);
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.state == TURN_RUNNING);
  assert(turn.target_deg > 89.5f && turn.target_deg < 90.5f);
  assert(TurnControl_IsActive() == 1U);

  /* 运行中重复启动被拒。 */
  assert(TurnControl_Start(90.0f) == HAL_BUSY);

  /* 首轮过程：误差 90 度，请求限幅到 800，左轮负右轮正（左转）。 */
  TurnControl_Process();
  assert(sc_settarget_count == 1U);
  assert(sc_left_target > -800.5f && sc_left_target < -799.5f);
  assert(sc_right_target > 799.5f && sc_right_target < 800.5f);
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.error_deg > 89.5f && turn.error_deg < 90.5f);

  /* 模拟以 60 deg/s 左转 1 秒：z 到约 60 度，误差 30，请求 360。 */
  FeedGyro(ticks, 0.0f);
  {
    uint32_t t;
    for (t = 0U; t < 100U; ++t)
    {
      ticks += 10U;
      FeedGyro(ticks, 60.0f);
      TurnControl_Process();
    }
  }
  {
    const Heading_Data *heading = Heading_GetData();
    assert(heading->z_deg > 59.0f && heading->z_deg < 61.0f);
    /* 误差 30 度、增益 20：请求 600 counts/s，高于低速钳位不受影响。 */
    assert(sc_left_target < -599.5f && sc_left_target > -600.5f);
    assert(sc_right_target > 599.5f && sc_right_target < 600.5f);
  }

  /* 继续转到误差进入 2 度容差：30 deg/s 再转约 28.5 度到 z 约 88.5。 */
  {
    uint32_t t;
    for (t = 0U; t < 95U; ++t)
    {
      ticks += 10U;
      FeedGyro(ticks, 30.0f);
      TurnControl_Process();
    }
  }
  {
    const Heading_Data *heading = Heading_GetData();
    /* z 已越过 90 度目标附近；误差进入容差后过程应继续微调而非过冲失控。 */
    assert(heading->z_deg > 88.5f);
  }
  /* 冻结旋转（角速度为零）让误差停在容差内并走完确认时间。 */
  {
    uint32_t t;
    for (t = 0U; t < 40U; ++t)
    {
      ticks += 10U;
      FeedGyro(ticks, 0.0f);
      TurnControl_Process();
      if (TurnControl_GetData(&turn) == HAL_OK && turn.state == TURN_DONE)
      {
        break;
      }
    }
    assert(turn.state == TURN_DONE);
    assert(sc_stop_called == 1U);
    assert(turn.error_deg < tolerance && turn.error_deg > -tolerance);
    assert(TurnControl_IsActive() == 0U);
  }

  /* 正常完成后可直接再启动右转 -90 度：目标基于当前 z 角度。 */
  sc_stop_called = 0U;
  assert(TurnControl_Start(-90.0f) == HAL_OK);
  assert(TurnControl_GetData(&turn) == HAL_OK);
  {
    const Heading_Data *heading = Heading_GetData();
    assert(turn.target_deg < heading->z_deg - 89.5f);
  }
  /* 右转首轮：误差 -90，请求 -800，左轮正右轮负。 */
  TurnControl_Process();
  assert(sc_left_target > 799.5f && sc_left_target < 800.5f);
  assert(sc_right_target < -799.5f && sc_right_target > -800.5f);
  /* 停止命令回到待机。 */
  TurnControl_Stop();
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.state == TURN_IDLE);

  /* 超时保护：右转 90 度预算约 5.7 s，喂静止样本阻止到位但不老化数据。 */
  assert(TurnControl_Start(-90.0f) == HAL_OK);
  TurnControl_Process();
  {
    uint32_t t;
    for (t = 0U; t < 700U; ++t)
    {
      ticks += 10U;
      FeedGyro(ticks, 0.0f);
      TurnControl_Process();
      if (TurnControl_GetData(&turn) == HAL_OK && turn.state != TURN_RUNNING)
      {
        break;
      }
    }
    assert(turn.state == TURN_FAULT);
    assert(sc_stop_called == 1U);
    /* 故障锁存：未 Stop 前拒绝再启动。 */
    assert(TurnControl_Start(90.0f) == HAL_ERROR);
    /* Stop 解除锁存。 */
    TurnControl_Stop();
    assert(TurnControl_Start(90.0f) == HAL_OK);
  }

  /* 陀螺仪数据失效保护：停止喂样本 200 ms 后锁存故障。 */
  TurnControl_Stop();
  Heading_Reset();
  ticks += 100U;
  FeedGyro(ticks, 0.0f);
  assert(TurnControl_Start(90.0f) == HAL_OK);
  TurnControl_Process();
  ticks += 210U; /* 无新样本，z 角度样本年龄超限。 */
  TurnControl_Process();
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.state == TURN_FAULT);
  TurnControl_Stop();

  /* 速度环使能失败时启动失败且不进入运行态。 */
  ResetStubs();
  Heading_Reset();
  ticks = 1000U;
  FeedGyro(ticks, 0.0f);
  sc_enable_result_ok = 0U;
  assert(TurnControl_Start(90.0f) == HAL_ERROR);
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.state == TURN_IDLE);

  /* SetTarget 被拒（目标非法等）时锁存故障并停机。 */
  sc_enable_result_ok = 1U;
  assert(TurnControl_Start(90.0f) == HAL_OK);
  sc_settarget_result = HAL_ERROR;
  TurnControl_Process();
  assert(TurnControl_GetData(&turn) == HAL_OK);
  assert(turn.state == TURN_FAULT);
  assert(sc_stop_called == 1U);
  TurnControl_Stop();

  printf("Turn control integration tests passed\n");
  return 0;
}
