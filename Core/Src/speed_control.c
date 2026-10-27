#include "speed_control.h"
#include "motor.h"

typedef struct
{
  float integral;
  float previous_measured;
  uint8_t history_valid;
} SpeedControl_Wheel;

static uint8_t initialized;
static SpeedControl_Config settings;
static SpeedControl_Data snapshot;
static SpeedControl_Wheel left_wheel;
static SpeedControl_Wheel right_wheel;

/* 利用有序比较同时排除 NaN、无穷大及不合理数值，兼容 ARMCC5。 */
static uint8_t SpeedControl_InRange(float value, float low, float high)
{
  return (uint8_t)(value >= low && value <= high);
}

/* 对积分、命令及输出做对称限幅。 */
static float SpeedControl_Clamp(float value, float limit)
{
  if (value > limit) { return limit; }
  if (value < -limit) { return -limit; }
  return value;
}

/* 清除单轮积分和微分历史，防止停机或换向后沿用旧控制量。 */
static void SpeedControl_ResetWheel(SpeedControl_Wheel *wheel)
{
  wheel->integral = 0.0f;
  wheel->previous_measured = 0.0f;
  wheel->history_valid = 0U;
}

/* 统一撤销驱动及旧命令；故障状态必须由上层显式 Stop 后才能解除。 */
static void SpeedControl_Halt(SpeedControl_State state, SpeedControl_Fault fault)
{
  Motor_Disable();
  SpeedControl_ResetWheel(&left_wheel);
  SpeedControl_ResetWheel(&right_wheel);
  snapshot.left_target = 0.0f;
  snapshot.right_target = 0.0f;
  snapshot.left_duty = 0;
  snapshot.right_duty = 0;
  snapshot.state = state;
  snapshot.fault = fault;
}

/* 限定增益以防浮点溢出；零增益允许用于初始调参，但不代表已经标定。 */
static uint8_t SpeedControl_ValidGains(const SpeedControl_Gains *gains)
{
  return (uint8_t)(SpeedControl_InRange(gains->kp, 0.0f, 1000.0f) &&
                   SpeedControl_InRange(gains->ki, 0.0f, 1000.0f) &&
                   SpeedControl_InRange(gains->kd, 0.0f, 1000.0f));
}

/* 共享参数校验，保证初始化与在线更新具有完全一致的边界。 */
static uint8_t SpeedControl_ValidConfig(const SpeedControl_Config *config)
{
  if (config == NULL || !SpeedControl_ValidGains(&config->left) ||
      !SpeedControl_ValidGains(&config->right) ||
      !SpeedControl_InRange(config->max_target_counts_per_s, 1.0f, 1000000.0f) ||
      config->max_duty < 1 || config->max_duty > MOTOR_DUTY_MAX)
  {
    return 0U;
  }
  return 1U;
}

/* 复制配置并锁住电机；控制器工作期间由本模块独占电机命令接口。 */
HAL_StatusTypeDef SpeedControl_Init(const SpeedControl_Config *config)
{
  if (initialized != 0U) { return HAL_BUSY; }
  if (!SpeedControl_ValidConfig(config)) { return HAL_ERROR; }
  settings = *config;
  initialized = 1U;
  SpeedControl_Halt(SPEED_CONTROL_IDLE, SPEED_CONTROL_FAULT_NONE);
  return HAL_OK;
}

/* 在线更新前先完整校验，避免只改动一侧或部分字段。 */
HAL_StatusTypeDef SpeedControl_Configure(const SpeedControl_Config *config)
{
  int16_t left_duty;
  int16_t right_duty;
  if (initialized == 0U || !SpeedControl_ValidConfig(config)) { return HAL_ERROR; }
  left_duty = (int16_t)SpeedControl_Clamp((float)snapshot.left_duty, (float)config->max_duty);
  right_duty = (int16_t)SpeedControl_Clamp((float)snapshot.right_duty, (float)config->max_duty);
  /* 先确认电机接受限幅后的命令，再提交配置；失败时旧参数仍然完整保留。 */
  if (snapshot.state == SPEED_CONTROL_RUNNING && Motor_SetDuty(left_duty, right_duty) != HAL_OK)
  {
    SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_MOTOR);
    return HAL_ERROR;
  }
  settings = *config;
  left_wheel.integral = (settings.left.ki == 0.0f) ? 0.0f :
    SpeedControl_Clamp(left_wheel.integral, (float)settings.max_duty);
  right_wheel.integral = (settings.right.ki == 0.0f) ? 0.0f :
    SpeedControl_Clamp(right_wheel.integral, (float)settings.max_duty);
  left_wheel.history_valid = right_wheel.history_valid = 0U;
  snapshot.left_target = SpeedControl_Clamp(snapshot.left_target, settings.max_target_counts_per_s);
  snapshot.right_target = SpeedControl_Clamp(snapshot.right_target, settings.max_target_counts_per_s);
  snapshot.left_duty = left_duty;
  snapshot.right_duty = right_duty;
  return HAL_OK;
}

/* 回读运行时参数；只返回副本，防止调用者绕过校验修改内部配置。 */
HAL_StatusTypeDef SpeedControl_GetConfig(SpeedControl_Config *config)
{
  if (initialized == 0U || config == NULL) { return HAL_ERROR; }
  *config = settings;
  return HAL_OK;
}

/* 以当前有效采样为基线，等待下一份采样后才允许第一次非零输出。 */
HAL_StatusTypeDef SpeedControl_Enable(void)
{
  Encoder_Data data;
  if (initialized == 0U || snapshot.state == SPEED_CONTROL_FAULT) { return HAL_ERROR; }
  if (snapshot.state == SPEED_CONTROL_RUNNING) { return HAL_OK; }
  if (Encoder_GetData(&data) != HAL_OK || data.sample_valid == 0U ||
      data.interval_ms == 0U || data.interval_ms >= SPEED_CONTROL_FEEDBACK_TIMEOUT_MS ||
      (uint32_t)(HAL_GetTick() - data.timestamp_ms) >= SPEED_CONTROL_FEEDBACK_TIMEOUT_MS)
  {
    return HAL_BUSY;
  }
  if (Motor_Enable() != HAL_OK)
  {
    SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_MOTOR);
    return HAL_ERROR;
  }
  snapshot.feedback_timestamp_ms = data.timestamp_ms;
  snapshot.state = SPEED_CONTROL_RUNNING;
  return HAL_OK;
}

/* 停机后目标和积分全部清零，重新使能也不会自动恢复运动。 */
void SpeedControl_Stop(void)
{
  if (initialized != 0U)
  {
    SpeedControl_Halt(SPEED_CONTROL_IDLE, SPEED_CONTROL_FAULT_NONE);
  }
}

/* 改变目标前处理零速和换向，避免旧积分使车轮继续转动。 */
HAL_StatusTypeDef SpeedControl_SetTarget(float left, float right)
{
  if (initialized == 0U) { return HAL_ERROR; }
  if (!SpeedControl_InRange(left, -1.0e30f, 1.0e30f) ||
      !SpeedControl_InRange(right, -1.0e30f, 1.0e30f))
  {
    if (snapshot.state == SPEED_CONTROL_RUNNING)
    {
      SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_COMMAND);
    }
    return HAL_ERROR;
  }
  if (snapshot.state != SPEED_CONTROL_RUNNING) { return HAL_BUSY; }
  left = SpeedControl_Clamp(left, settings.max_target_counts_per_s);
  right = SpeedControl_Clamp(right, settings.max_target_counts_per_s);
  if (left == 0.0f || (left > 0.0f && snapshot.left_target < 0.0f) ||
      (left < 0.0f && snapshot.left_target > 0.0f))
  {
    SpeedControl_ResetWheel(&left_wheel);
    snapshot.left_duty = 0;
  }
  if (right == 0.0f || (right > 0.0f && snapshot.right_target < 0.0f) ||
      (right < 0.0f && snapshot.right_target > 0.0f))
  {
    SpeedControl_ResetWheel(&right_wheel);
    snapshot.right_duty = 0;
  }
  snapshot.left_target = left;
  snapshot.right_target = right;
  if (Motor_SetDuty(snapshot.left_duty, snapshot.right_duty) != HAL_OK)
  {
    SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_MOTOR);
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* 位置式 PID：微分作用于测量值；饱和时仅允许能退出饱和的积分变化。
   输出仅沿目标方向驱动，超速时滑行，不施加反向制动力。 */
static int16_t SpeedControl_Calculate(SpeedControl_Wheel *wheel,
                                    const SpeedControl_Gains *gains,
                                    float target, float measured, float dt)
{
  float error;
  float derivative = 0.0f;
  float candidate;
  float base;
  float output;
  float low = (target > 0.0f) ? 0.0f : -(float)settings.max_duty;
  float high = (target > 0.0f) ? (float)settings.max_duty : 0.0f;
  if (target == 0.0f)
  {
    SpeedControl_ResetWheel(wheel);
    return 0;
  }
  error = target - measured;
  if (wheel->history_valid != 0U)
  {
    derivative = -gains->kd * (measured - wheel->previous_measured) / dt;
  }
  candidate = SpeedControl_Clamp(wheel->integral + gains->ki * error * dt,
                                (float)settings.max_duty);
  base = gains->kp * error + derivative;
  output = base + candidate;
  /* 输出触顶且误差继续推高时冻结积分；触底时采取对称处理。 */
  if (!((output > high && error > 0.0f) || (output < low && error < 0.0f)))
  {
    wheel->integral = candidate;
  }
  output = base + wheel->integral;
  if (output > high) { output = high; }
  if (output < low) { output = low; }
  wheel->previous_measured = measured;
  wheel->history_valid = 1U;
  return (int16_t)output;
}

/* 连续监测反馈年龄；同一采样只消费一次，迟到采样不能掩盖此前超时。 */
HAL_StatusTypeDef SpeedControl_Update(void)
{
  Encoder_Data data;
  float dt;
  if (initialized == 0U || snapshot.state == SPEED_CONTROL_FAULT) { return HAL_ERROR; }
  if (snapshot.state != SPEED_CONTROL_RUNNING) { return HAL_BUSY; }
  if ((uint32_t)(HAL_GetTick() - snapshot.feedback_timestamp_ms) >=
      SPEED_CONTROL_FEEDBACK_TIMEOUT_MS || Encoder_GetData(&data) != HAL_OK ||
      data.sample_valid == 0U || data.interval_ms == 0U ||
      data.interval_ms >= SPEED_CONTROL_FEEDBACK_TIMEOUT_MS ||
      (uint32_t)(HAL_GetTick() - data.timestamp_ms) >= SPEED_CONTROL_FEEDBACK_TIMEOUT_MS ||
      !SpeedControl_InRange(data.left_counts_per_s, -3276800.0f, 3276800.0f) ||
      !SpeedControl_InRange(data.right_counts_per_s, -3276800.0f, 3276800.0f))
  {
    SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_FEEDBACK);
    return HAL_ERROR;
  }
  if (data.timestamp_ms == snapshot.feedback_timestamp_ms) { return HAL_BUSY; }
  dt = (float)data.interval_ms * 0.001f;
  snapshot.left_measured = data.left_counts_per_s;
  snapshot.right_measured = data.right_counts_per_s;
  snapshot.feedback_timestamp_ms = data.timestamp_ms;
  snapshot.left_duty = SpeedControl_Calculate(&left_wheel, &settings.left,
                       snapshot.left_target, snapshot.left_measured, dt);
  snapshot.right_duty = SpeedControl_Calculate(&right_wheel, &settings.right,
                        snapshot.right_target, snapshot.right_measured, dt);
  if (Motor_SetDuty(snapshot.left_duty, snapshot.right_duty) != HAL_OK)
  {
    SpeedControl_Halt(SPEED_CONTROL_FAULT, SPEED_CONTROL_FAULT_MOTOR);
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* 状态快照与控制过程均在主循环访问，无需在中断中复制浮点数据。 */
HAL_StatusTypeDef SpeedControl_GetData(SpeedControl_Data *data)
{
  if (initialized == 0U || data == NULL) { return HAL_ERROR; }
  *data = snapshot;
  return HAL_OK;
}
