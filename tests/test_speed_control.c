#include "speed_control.h"
#include "motor.h"
#include "tim.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim1, htim2, htim3, htim4;
static TIM_TypeDef pwm_right, pwm_left, encoder_left, encoder_right;
static uint32_t ticks;

/* 使用可控毫秒时钟，覆盖采样延迟、超时及无符号回绕。 */
uint32_t HAL_GetTick(void) { return ticks; }

/* 启动真实电机驱动使用的模拟 PWM，不连接实物。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer;
  (void)channel;
  return HAL_OK;
}

/* 启动真实编码器模块使用的模拟计数器。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer;
  (void)channel;
  return HAL_OK;
}

/* 提供编码器初始化回滚路径所需的 HAL 接口。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer;
  (void)channel;
  return HAL_OK;
}

/* 输入车轮逻辑方向的计数增量，按安装极性还原硬件计数后采样。
   方向标定的物理回放由 test_encoder.c 的 forward 分支独立验证。 */
static HAL_StatusTypeDef Sample(uint32_t elapsed, int left, int right)
{
  ticks += elapsed;
  encoder_left.CNT = (uint16_t)(encoder_left.CNT + left * ENCODER_LEFT_SIGN);
  encoder_right.CNT = (uint16_t)(encoder_right.CNT + right * ENCODER_RIGHT_SIGN);
  assert(Encoder_Update() == HAL_OK);
  return SpeedControl_Update();
}

/* 读取控制快照，所有断言同时验证公开接口可以正常观察状态。 */
static SpeedControl_Data Data(void)
{
  SpeedControl_Data data;
  assert(SpeedControl_GetData(&data) == HAL_OK);
  return data;
}

/* 故障与停机必须把真实驱动的四路 CCR 全部撤销。 */
static void CheckStopped(void)
{
  assert(pwm_left.CCR3 == 0U && pwm_left.CCR4 == 0U);
  assert(pwm_right.CCR1 == 0U && pwm_right.CCR2 == 0U);
  assert(Data().left_duty == 0 && Data().right_duty == 0);
  assert(Data().left_target == 0.0f && Data().right_target == 0.0f);
}

/* 运行真实编码器、电机和控制器，验证闭环接口、保护及积分行为。
   derivative 参数在独立进程中启用纯微分，避免测试依赖内部状态修改。 */
int main(int argc, char **argv)
{
  SpeedControl_Config config = {{1.0f, 10.0f, 0.0f}, {1.0f, 10.0f, 0.0f}, 3000.0f, 1000};
  uint8_t derivative = (uint8_t)(argc > 1 && strcmp(argv[1], "derivative") == 0);
  htim1.Instance = &pwm_right;
  htim2.Instance = &pwm_left;
  htim3.Instance = &encoder_left;
  htim4.Instance = &encoder_right;
  htim1.Init.Period = htim2.Init.Period = 3599U;
  htim1.Init.RepetitionCounter = 19U;
  htim3.Init.Period = htim4.Init.Period = 65535U;
  assert(SpeedControl_Update() == HAL_ERROR);
  assert(SpeedControl_Enable() == HAL_ERROR);
  assert(SpeedControl_SetTarget(1.0f, 1.0f) == HAL_ERROR);
  assert(Motor_Init() == HAL_OK && Encoder_Init() == HAL_OK);
  assert(SpeedControl_Init(NULL) == HAL_ERROR);
  config.left.kp = NAN;
  assert(SpeedControl_Init(&config) == HAL_ERROR);
  config.left.kp = 1.0f;
  config.max_duty = 1001;
  assert(SpeedControl_Init(&config) == HAL_ERROR);
  config.max_duty = 1000;
  if (derivative != 0U)
  {
    config.left.kp = config.left.ki = 0.0f;
    config.left.kd = 1.0f;
  }
  assert(SpeedControl_Init(&config) == HAL_OK);
  assert(SpeedControl_Init(&config) == HAL_BUSY);
  assert(SpeedControl_GetData(NULL) == HAL_ERROR);
  assert(Data().state == SPEED_CONTROL_IDLE);
  CheckStopped();
  assert(SpeedControl_SetTarget(100.0f, 100.0f) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_BUSY);
  assert(Sample(10U, 0, 0) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(100.0f, -100.0f) == HAL_OK);
  assert(SpeedControl_Enable() == HAL_OK);
  assert(Data().left_target == 100.0f);
  /* 使能时的旧采样不参与积分，也不产生非零 PWM。 */
  assert(SpeedControl_Update() == HAL_BUSY);
  assert(pwm_left.CCR4 == 0U);

  if (derivative != 0U)
  {
    assert(Sample(10U, 1, 0) == HAL_OK);
    assert(Data().left_duty == 0);
    assert(Sample(20U, 1, 0) == HAL_OK);
    assert(Data().left_duty == 1000);
    assert(SpeedControl_SetTarget(200.0f, 0.0f) == HAL_OK);
    assert(Sample(20U, 1, 0) == HAL_OK);
    assert(Data().left_duty == 0);
    puts("Speed measurement derivative tests passed");
    return 0;
  }

  assert(Sample(10U, 0, 0) == HAL_OK);
  assert(Data().left_duty == 110 && Data().right_duty == -110);
  assert(pwm_left.CCR4 > 0U && pwm_right.CCR2 > 0U);
  assert(SpeedControl_Update() == HAL_BUSY && Data().left_duty == 110);
  /* 20 ms 实际间隔产生两倍积分变化，不假定每次都是 10 ms。 */
  assert(Sample(20U, 0, 0) == HAL_OK);
  assert(Data().left_duty == 130 && Data().right_duty == -130);
  assert(SpeedControl_SetTarget(0.0f, 0.0f) == HAL_OK);
  CheckStopped();
  assert(SpeedControl_SetTarget(10.0f, 0.0f) == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == 11);
  /* 饱和期间冻结积分，降低目标后不能被旧的大积分继续推到满输出。 */
  assert(SpeedControl_SetTarget(9999.0f, -9999.0f) == HAL_OK);
  assert(Data().left_target == 3000.0f && Data().right_target == -3000.0f);
  assert(Sample(10U, 0, 0) == HAL_OK);
  assert(Data().left_duty == 1000 && Data().right_duty == -1000);
  assert(Sample(10U, 0, 0) == HAL_OK);
  assert(SpeedControl_SetTarget(10.0f, 0.0f) == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == 12);
  /* 单轮换向立即清空该轮输出，下一采样再按新方向重新计算。 */
  assert(SpeedControl_SetTarget(-100.0f, 0.0f) == HAL_OK);
  assert(Data().left_duty == 0 && pwm_left.CCR4 == 0U);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == -110);
  /* 超速时滑行，不能施加反向 PWM。 */
  assert(Sample(10U, -10, 0) == HAL_OK && Data().left_duty == 0);
  ticks += 49U;
  assert(SpeedControl_Update() == HAL_BUSY);
  ++ticks;
  assert(SpeedControl_Update() == HAL_ERROR);
  assert(Data().state == SPEED_CONTROL_FAULT && Data().fault == SPEED_CONTROL_FAULT_FEEDBACK);
  CheckStopped();
  assert(SpeedControl_Enable() == HAL_ERROR);
  assert(Sample(10U, 0, 0) == HAL_ERROR);
  SpeedControl_Stop();
  /* 新采样的时间戳再新，跨度过大的平均速度也不能作为启动反馈。 */
  assert(SpeedControl_Enable() == HAL_BUSY);
  assert(Sample(10U, 0, 0) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(100.0f, 100.0f) == HAL_OK);
  assert(Sample(50U, 0, 0) == HAL_ERROR);
  CheckStopped();
  SpeedControl_Stop();
  assert(Sample(10U, 0, 0) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_OK);
  /* 编码器半范围歧义立即触发保护，不使用残留计数率。 */
  ticks += 10U;
  encoder_left.CNT = (uint16_t)(encoder_left.CNT + 32768U);
  assert(Encoder_Update() == HAL_ERROR);
  assert(SpeedControl_Update() == HAL_ERROR);
  CheckStopped();
  SpeedControl_Stop();
  assert(Sample(10U, 0, 0) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(NAN, 0.0f) == HAL_ERROR);
  assert(Data().fault == SPEED_CONTROL_FAULT_COMMAND);
  CheckStopped();
  SpeedControl_Stop();
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(INFINITY, 0.0f) == HAL_ERROR);
  CheckStopped();
  SpeedControl_Stop();
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(100.0f, 100.0f) == HAL_OK);
  Motor_Disable();
  assert(Sample(10U, 0, 0) == HAL_ERROR);
  assert(Data().fault == SPEED_CONTROL_FAULT_MOTOR);
  CheckStopped();
  SpeedControl_Stop();
  /* 跨越 UINT32_MAX 时，新反馈仍可驱动控制；超时仍按经过时间判断。 */
  ticks = 0xFFFFFFF0U;
  assert(Encoder_Reset() == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_BUSY);
  assert(SpeedControl_Enable() == HAL_OK);
  assert(SpeedControl_SetTarget(100.0f, 0.0f) == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == 110);
  SpeedControl_Stop();
  CheckStopped();
  assert(SpeedControl_Enable() == HAL_OK && Data().left_target == 0.0f);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == 0);
  /* 在线调参保留运行状态，降低限幅立即约束目标和输出。 */
  assert(SpeedControl_GetConfig(NULL) == HAL_ERROR);
  assert(SpeedControl_GetConfig(&config) == HAL_OK);
  assert(SpeedControl_SetTarget(500.0f, 500.0f) == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty > 50);
  config.max_duty = 50;
  config.max_target_counts_per_s = 20.0f;
  assert(SpeedControl_Configure(&config) == HAL_OK);
  assert(Data().state == SPEED_CONTROL_RUNNING && Data().left_target == 20.0f);
  assert(Data().left_duty <= 50 && Data().right_duty <= 50);
  config.left.kp = NAN;
  assert(SpeedControl_Configure(&config) == HAL_ERROR);
  assert(SpeedControl_GetConfig(&config) == HAL_OK && config.left.kp == 1.0f);
  config.left.kp = 0.0f;
  config.left.ki = 0.0f;
  assert(SpeedControl_Configure(&config) == HAL_OK);
  assert(Sample(10U, 0, 0) == HAL_OK && Data().left_duty == 0);
  SpeedControl_Stop();
  /* 电机拒绝调参时不提交新配置，并保留明确的电机故障状态。 */
  assert(SpeedControl_Enable() == HAL_OK);
  Motor_Disable();
  config.max_duty = 40;
  assert(SpeedControl_Configure(&config) == HAL_ERROR);
  assert(SpeedControl_GetConfig(&config) == HAL_OK && config.max_duty == 50);
  assert(Data().fault == SPEED_CONTROL_FAULT_MOTOR);
  puts("Speed control integration and safety tests passed");
  return 0;
}
