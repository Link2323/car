#include "turn_control.h"
#include "speed_control.h"
#include "heading.h"

/* 差速轮速比例增益：counts/s 每度剩余误差。90 度误差时请求 1800，
   被 800 上限截住，误差缩到约 40 度内开始线性减速。 */
#define TURN_KP_COUNTS_PER_DEG 20.0f
/* 单轮差速速度上限，counts/s；800 约 40 mm/s 轮速，首轮架空实测确认。 */
#define TURN_MAX_COUNTS_PER_S 800.0f
/* 低速钳位：误差接近容差时比例请求已小到克服不了轮子静摩擦，
   实测 72 counts/s 轮子停转、150 counts/s 地面原地转仍打滑蠕动，
   卡在容差外直到超时；抬高到该值保持转向动力（约 12.5 mm/s 轮速，
   低速直行测试已验证可动），靠容差判定与反向修正收敛。 */
#define TURN_MIN_COUNTS_PER_S 250.0f
/* 到位容差与确认时间：2.5 度容差覆盖低速段打滑残余误差。确认时间
   从 300ms 缩到 50ms——180 度掉头高位速接近时，容差内零速等确认
   期间惯性滑出容差、恢复修正再滑出的循环会让车在目标附近打转
   直到超时；一进容差尽快完成，残余过冲由下一段直行的航向基准
   （取完成时刻实际 z 角度）自然吸收。 */
#define TURN_TOLERANCE_DEG 2.5f
#define TURN_SETTLE_MS 50U
/* 超时 = 基础时间 + 每度余量；90 度约 5.7 s、180 度约 8.4 s。 */
#define TURN_TIMEOUT_BASE_MS 3000U
#define TURN_TIMEOUT_MS_PER_DEG 30U
/* z 角度样本超过该时长未刷新视为陀螺仪数据失效，立即停机。 */
#define TURN_GYRO_STALE_MS 200U
/* 允许的相对转向角范围（度）。 */
#define TURN_MAX_DELTA_DEG 360.0f

static TurnControl_Data turn;
static uint32_t started_ms;
static uint32_t settle_started_ms;
static uint32_t timeout_ms;
static uint8_t initialized;

/* 清到位确认窗口；进入误差容差的时刻重新计时。 */
static void TurnControl_ResetSettle(void)
{
  settle_started_ms = 0U;
}

/* 初始化快照；重复调用返回忙，防止运行中意外重置状态。 */
HAL_StatusTypeDef TurnControl_Init(void)
{
  if (initialized != 0U) { return HAL_BUSY; }
  turn.state = TURN_IDLE;
  turn.target_deg = 0.0f;
  turn.error_deg = 0.0f;
  turn.speed_counts_per_s = 0.0f;
  turn.elapsed_ms = 0U;
  TurnControl_ResetSettle();
  initialized = 1U;
  return HAL_OK;
}

/* 启动一次相对转向：以当前 z 角度为基准换算绝对目标并使能速度环。
   待机或上次正常完成时可直接再启动；故障锁存需先 Stop 复位。 */
HAL_StatusTypeDef TurnControl_Start(float delta_deg)
{
  const Heading_Data *heading;
  if (initialized == 0U) { return HAL_ERROR; }
  if (delta_deg == 0.0f || delta_deg > TURN_MAX_DELTA_DEG ||
      delta_deg < -TURN_MAX_DELTA_DEG)
  {
    return HAL_ERROR;
  }
  if (turn.state == TURN_RUNNING) { return HAL_BUSY; }
  if (turn.state == TURN_FAULT) { return HAL_ERROR; }
  heading = Heading_GetData();
  if (heading == NULL || heading->valid == 0U)
  {
    return HAL_ERROR;
  }
  /* 使能速度环（要求编码器反馈新鲜），目标由 Process 首轮下发。 */
  if (SpeedControl_Enable() != HAL_OK) { return HAL_ERROR; }
  started_ms = HAL_GetTick();
  turn.target_deg = heading->z_deg + delta_deg;
  turn.error_deg = delta_deg;
  turn.speed_counts_per_s = 0.0f;
  turn.elapsed_ms = 0U;
  TurnControl_ResetSettle();
  timeout_ms = TURN_TIMEOUT_BASE_MS +
               (uint32_t)((delta_deg >= 0.0f ? delta_deg : -delta_deg) *
                          (float)TURN_TIMEOUT_MS_PER_DEG);
  turn.state = TURN_RUNNING;
  return HAL_OK;
}

/* 停止速度环输出并回到待机；同时清除转向故障锁存，允许重新启动。 */
void TurnControl_Stop(void)
{
  SpeedControl_Stop();
  turn.state = TURN_IDLE;
  TurnControl_ResetSettle();
}

/* 转向主逻辑：误差比例控制 + 到位确认 + 超时与数据失效保护。 */
void TurnControl_Process(void)
{
  const Heading_Data *heading;
  SpeedControl_Data speed;
  uint32_t now;
  float error;
  float request;
  float magnitude;

  if (initialized == 0U || turn.state != TURN_RUNNING) { return; }
  now = HAL_GetTick();
  /* 先查数据健康：z 角度无效或样本过旧时立即锁存故障并停机。 */
  heading = Heading_GetData();
  if (heading == NULL || heading->valid == 0U ||
      (now - heading->last_sample_ms) > TURN_GYRO_STALE_MS)
  {
    SpeedControl_Stop();
    turn.state = TURN_FAULT;
    TurnControl_ResetSettle();
    return;
  }
  /* 速度环自身故障（反馈异常等）同样锁存，保留其故障码供诊断。 */
  if (SpeedControl_GetData(&speed) != HAL_OK ||
      speed.state == SPEED_CONTROL_FAULT)
  {
    turn.state = TURN_FAULT;
    TurnControl_ResetSettle();
    return;
  }
  turn.elapsed_ms = now - started_ms;
  /* 超时保护：剩余时间预算用尽仍未到位，停机并锁存故障。 */
  if (turn.elapsed_ms > timeout_ms)
  {
    SpeedControl_Stop();
    turn.state = TURN_FAULT;
    TurnControl_ResetSettle();
    return;
  }
  error = turn.target_deg - heading->z_deg;
  turn.error_deg = error;
  /* 到位判定：进入容差后连续保持 300 ms 才算完成，抗误差零点穿越。
     容差内立即输出零速：若仍按低速钳位继续蠕动，车会在容差边缘
     来回蹭动（末端顿挫），停住等待确认即可，滑出容差自然恢复修正。 */
  magnitude = (error >= 0.0f) ? error : -error;
  if (magnitude < TURN_TOLERANCE_DEG)
  {
    if (settle_started_ms == 0U) { settle_started_ms = now; }
    if ((uint32_t)(now - settle_started_ms) >= TURN_SETTLE_MS)
    {
      SpeedControl_Stop();
      turn.state = TURN_DONE;
      turn.speed_counts_per_s = 0.0f;
      TurnControl_ResetSettle();
      return;
    }
    turn.speed_counts_per_s = 0.0f;
    if (SpeedControl_SetTarget(0.0f, 0.0f) != HAL_OK)
    {
      SpeedControl_Stop();
      turn.state = TURN_FAULT;
      TurnControl_ResetSettle();
    }
    return;
  }
  TurnControl_ResetSettle();
  /* 比例控制输出差速请求：左轮 -v、右轮 +v，v 正值左转（z 增大）；
     双向限幅后对小请求做低速钳位，避免末端轮速过低被静摩擦卡停。 */
  request = TURN_KP_COUNTS_PER_DEG * error;
  if (request > TURN_MAX_COUNTS_PER_S) { request = TURN_MAX_COUNTS_PER_S; }
  if (request < -TURN_MAX_COUNTS_PER_S) { request = -TURN_MAX_COUNTS_PER_S; }
  if (request > 0.0f && request < TURN_MIN_COUNTS_PER_S)
  {
    request = TURN_MIN_COUNTS_PER_S;
  }
  else if (request < 0.0f && request > -TURN_MIN_COUNTS_PER_S)
  {
    request = -TURN_MIN_COUNTS_PER_S;
  }
  turn.speed_counts_per_s = request;
  if (SpeedControl_SetTarget(-request, request) != HAL_OK)
  {
    SpeedControl_Stop();
    turn.state = TURN_FAULT;
    TurnControl_ResetSettle();
  }
}

uint8_t TurnControl_IsActive(void)
{
  return (uint8_t)(turn.state == TURN_RUNNING);
}

HAL_StatusTypeDef TurnControl_GetData(TurnControl_Data *data)
{
  if (data == NULL) { return HAL_ERROR; }
  *data = turn;
  return HAL_OK;
}
