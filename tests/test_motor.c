#include "motor.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
static TIM_TypeDef right_timer;
static TIM_TypeDef left_timer;
static uint32_t ticks;
static unsigned int start_count;
static unsigned int fail_start_at;

/* 提供可控节拍，以毫秒边界和回绕验证非阻塞换向状态机。 */
uint32_t HAL_GetTick(void)
{
  return ticks;
}

/* 模拟 PWM 启动，任何一个通道启动前四路 CCR 都必须为零。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  assert((timer == &htim1 && (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2)) ||
         (timer == &htim2 && (channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4)));
  assert(right_timer.CCR1 == 0U && right_timer.CCR2 == 0U);
  assert(left_timer.CCR3 == 0U && left_timer.CCR4 == 0U);
  ++start_count;
  return (start_count == fail_start_at) ? HAL_ERROR : HAL_OK;
}

/* 按物理 BI/FI 检查输出；右轮正命令使用 BI，负命令使用 FI。
   同时检查输入互斥，不允许同一轮的 BI/FI 同时非零。 */
static void CheckOutputs(uint32_t left_bi, uint32_t left_fi,
                         uint32_t right_bi, uint32_t right_fi)
{
  assert(left_timer.CCR3 == left_bi && left_timer.CCR4 == left_fi);
  assert(right_timer.CCR1 == right_bi && right_timer.CCR2 == right_fi);
  assert(left_bi == 0U || left_fi == 0U);
  assert(right_bi == 0U || right_fi == 0U);
}

/* 在指定节拍服务一次状态机，不依赖真实延时或任何 ARM 外设。 */
static void ProcessAt(uint32_t now)
{
  ticks = now;
  Motor_Process();
}

/* 测试初始化、显式使能、限幅、独立换向、取消命令及节拍回绕。 */
int main(int argc, char **argv)
{
  (void)argv;
  htim1.Instance = &right_timer;
  htim2.Instance = &left_timer;
  htim1.Init.Period = 3599U;
  htim2.Init.Period = 3599U;
  htim1.Init.RepetitionCounter = 19U;
  Motor_Stop();
  Motor_Disable();
  assert(Motor_Enable() == HAL_ERROR);
  assert(Motor_SetDuty(100, 100) == HAL_ERROR);

  /* 单独进程注入第二通道启动失败，避免上一次测试初始化状态影响结果。 */
  if (argc > 1)
  {
    fail_start_at = 2U;
    assert(Motor_Init() == HAL_ERROR);
    assert(Motor_Enable() == HAL_ERROR);
    CheckOutputs(0U, 0U, 0U, 0U);
    puts("Motor partial-start failure test passed");
    return 0;
  }

  htim1.Init.Period = 999U;
  assert(Motor_Init() == HAL_ERROR && start_count == 0U);
  htim1.Init.Period = 3599U;
  assert(Motor_Init() == HAL_OK && start_count == 4U);
  assert(Motor_Init() == HAL_BUSY);
  assert(Motor_SetDuty(500, -250) == HAL_BUSY);
  ProcessAt(100U);
  CheckOutputs(0U, 0U, 0U, 0U);

  ticks = 0U;
  assert(Motor_Enable() == HAL_OK);
  assert(Motor_SetDuty(500, -250) == HAL_OK);
  ProcessAt(4U);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(5U);
  CheckOutputs(0U, 1800U, 0U, 900U);
  assert(Motor_Enable() == HAL_OK);
  CheckOutputs(0U, 1800U, 0U, 900U);

  assert(Motor_SetDuty(-500, 250) == HAL_OK);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(9U);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(10U);
  CheckOutputs(1800U, 0U, 900U, 0U);

  /* 一轮换向时另一轮可继续同向调速，且极值命令正确饱和至 100%。 */
  assert(Motor_SetDuty(-32768, 32767) == HAL_OK);
  CheckOutputs(3600U, 0U, 3600U, 0U);
  assert(Motor_SetDuty(-1, 999) == HAL_OK);
  CheckOutputs(8U, 0U, 3592U, 0U);
  assert(Motor_SetDuty(-500, -500) == HAL_OK);
  CheckOutputs(1800U, 0U, 0U, 0U);
  Motor_Stop();
  ProcessAt(20U);
  CheckOutputs(0U, 0U, 0U, 0U);

  /* Stop 后立即反向仍须经过空档，不可通过零命令绕过换向保护。 */
  assert(Motor_SetDuty(-500, 250) == HAL_OK);
  Motor_Stop();
  assert(Motor_SetDuty(500, -250) == HAL_OK);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(24U);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(25U);
  CheckOutputs(0U, 1800U, 0U, 900U);

  assert(Motor_SetDuty(-500, 250) == HAL_OK);
  Motor_Disable();
  assert(Motor_SetDuty(1000, 1000) == HAL_BUSY);
  ProcessAt(100U);
  assert(Motor_Enable() == HAL_OK);
  ProcessAt(110U);
  CheckOutputs(0U, 0U, 0U, 0U);

  Motor_Disable();
  ticks = 0xFFFFFFFDU;
  assert(Motor_Enable() == HAL_OK);
  assert(Motor_SetDuty(500, -250) == HAL_OK);
  ProcessAt(1U);
  CheckOutputs(0U, 0U, 0U, 0U);
  ProcessAt(2U);
  CheckOutputs(0U, 1800U, 0U, 900U);
  Motor_Stop();
  CheckOutputs(0U, 0U, 0U, 0U);
  puts("Motor state-machine tests passed");
  return 0;
}
