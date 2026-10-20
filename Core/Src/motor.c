#include "motor.h"
#include "tim.h"

/* 沿用当前工程 72 MHz / 3600 = 20 kHz；最短脉冲 8 个计数约 111 ns。
   SA8345-MS 要求 PWM 高、低时间都不少于 100 ns，0%/100% 恒电平除外。 */
#define MOTOR_PWM_PERIOD_TICKS 3600U
#define MOTOR_MIN_PULSE_TICKS 8U

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t bi_channel;
  uint32_t fi_channel;
  /* 安装方向校正：+1 表示正命令驱动 FI，-1 表示正命令驱动 BI。 */
  int8_t polarity;
  int16_t target;
  int16_t applied;
  int8_t last_direction;
  uint32_t coast_since;
} Motor_Channel;

static Motor_Channel left_motor = {&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 1, 0, 0, 0, 0U};
/* 实测右轮在 FI 驱动时反转，因此仅将右轮的物理输出方向反向。 */
static Motor_Channel right_motor = {&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, -1, 0, 0, 0, 0U};
static uint8_t initialized;
static uint8_t enabled;
static uint32_t enabled_since;

/* 限制命令幅值，先在有符号域内比较，兼容 INT16_MIN 等边界输入。 */
static int16_t Motor_ClampDuty(int16_t duty)
{
  if (duty > MOTOR_DUTY_MAX)
  {
    return MOTOR_DUTY_MAX;
  }
  if (duty < -MOTOR_DUTY_MAX)
  {
    return -MOTOR_DUTY_MAX;
  }
  return duty;
}

/* 将占空比换算为 CCR；100% 使用 ARR+1，避免最大值仍有一个低电平计数。 */
static uint32_t Motor_DutyToCompare(int16_t duty)
{
  uint32_t magnitude = (uint32_t)((duty < 0) ? -(int32_t)duty : duty);
  uint32_t compare = magnitude * MOTOR_PWM_PERIOD_TICKS / MOTOR_DUTY_MAX;

  /* 中间占空比需满足芯片最小高低脉宽，避免极小脉冲不符合输入时序。 */
  if (magnitude != 0U && magnitude != MOTOR_DUTY_MAX)
  {
    if (compare < MOTOR_MIN_PULSE_TICKS)
    {
      compare = MOTOR_MIN_PULSE_TICKS;
    }
    if (compare > MOTOR_PWM_PERIOD_TICKS - MOTOR_MIN_PULSE_TICKS)
    {
      compare = MOTOR_PWM_PERIOD_TICKS - MOTOR_MIN_PULSE_TICKS;
    }
  }
  return compare;
}

/* 将某一轮双输入清零；保留最后驱动方向，以保护快速停机后立即反向的情况。 */
static void Motor_Coast(Motor_Channel *motor, uint32_t now)
{
  __HAL_TIM_SET_COMPARE(motor->timer, motor->bi_channel, 0U);
  __HAL_TIM_SET_COMPARE(motor->timer, motor->fi_channel, 0U);
  if (motor->applied != 0)
  {
    motor->coast_since = now;
    motor->applied = 0;
  }
}

/* 更新一轮的输出状态；换向先清零再等待，空档由主循环驱动，不阻塞 CPU。 */
static void Motor_ProcessChannel(Motor_Channel *motor, uint32_t now)
{
  int8_t direction;
  uint32_t compare;

  if (motor->target == 0)
  {
    Motor_Coast(motor, now);
    return;
  }
  /* 先换算为物理驱动方向；换向保护也使用该方向，目标占空比保持原值。 */
  direction = (int8_t)(((motor->target > 0) ? 1 : -1) * motor->polarity);
  /* 即便经过 Stop，也检查上一次实际输出方向，防止绕过换向等待。 */
  if (motor->last_direction != 0 && direction != motor->last_direction)
  {
    Motor_Coast(motor, now);
    if ((uint32_t)(now - motor->coast_since) < MOTOR_REVERSE_DELAY_MS)
    {
      return;
    }
  }

  compare = Motor_DutyToCompare(motor->target);
  /* 先保持非驱动通道为零，每次最多只让 BI 或 FI 中的一路输出 PWM。 */
  if (direction > 0)
  {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->bi_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->fi_channel, compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->fi_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->bi_channel, compare);
  }
  motor->applied = motor->target;
  motor->last_direction = direction;
}

/* 初始化仅启动零占空比 PWM，不接收或自动执行任何转动命令。 */
HAL_StatusTypeDef Motor_Init(void)
{
  if (initialized != 0U)
  {
    return HAL_BUSY;
  }
  /* 状态机的空档时间依据当前 20 kHz PWM 和 TIM1 的 1 ms 更新周期设计。
     配置被改动时拒绝初始化，避免沿用错误的 CCR 比例或换向延时假设。 */
  if (htim1.Init.Period != MOTOR_PWM_PERIOD_TICKS - 1U ||
      htim2.Init.Period != MOTOR_PWM_PERIOD_TICKS - 1U ||
      htim1.Init.Prescaler != 0U || htim2.Init.Prescaler != 0U ||
      htim1.Init.RepetitionCounter != 19U)
  {
    return HAL_ERROR;
  }
  Motor_Coast(&left_motor, HAL_GetTick());
  Motor_Coast(&right_motor, HAL_GetTick());
  /* 任一通道启动失败时保持全部 CCR 为零，主程序应进入错误处理。 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }
  enabled = 0U;
  initialized = 1U;
  return HAL_OK;
}

/* 显式打开命令入口，预留驱动电源稳定等待；函数本身不输出非零 PWM。 */
HAL_StatusTypeDef Motor_Enable(void)
{
  if (initialized == 0U)
  {
    return HAL_ERROR;
  }
  if (enabled == 0U)
  {
    left_motor.target = 0;
    right_motor.target = 0;
    enabled_since = HAL_GetTick();
    enabled = 1U;
  }
  return HAL_OK;
}

/* 清除两轮目标及待执行命令；CCR 预装载在各自下一次更新事件生效。 */
void Motor_Stop(void)
{
  uint32_t now;
  if (initialized == 0U)
  {
    return;
  }
  now = HAL_GetTick();
  left_motor.target = 0;
  right_motor.target = 0;
  Motor_Coast(&left_motor, now);
  Motor_Coast(&right_motor, now);
}

/* 禁用后不保存转动请求，重新使能也不会恢复旧的占空比。 */
void Motor_Disable(void)
{
  enabled = 0U;
  Motor_Stop();
}

/* 保存双轮限幅后的命令，并立即处理无需等待的同向调速或停机请求。 */
HAL_StatusTypeDef Motor_SetDuty(int16_t left, int16_t right)
{
  if (initialized == 0U)
  {
    return HAL_ERROR;
  }
  if (enabled == 0U)
  {
    return HAL_BUSY;
  }
  left_motor.target = Motor_ClampDuty(left);
  right_motor.target = Motor_ClampDuty(right);
  Motor_Process();
  return HAL_OK;
}

/* 持续服务双轮状态机；不在中断中调用，也不修改 TIM1 的计数器或更新事件。 */
void Motor_Process(void)
{
  uint32_t now;
  if (initialized == 0U || enabled == 0U)
  {
    return;
  }
  now = HAL_GetTick();
  if ((uint32_t)(now - enabled_since) < MOTOR_START_DELAY_MS)
  {
    return;
  }
  Motor_ProcessChannel(&left_motor, now);
  Motor_ProcessChannel(&right_motor, now);
}
