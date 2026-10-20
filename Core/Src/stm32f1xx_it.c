

#include "main.h"
#include "stm32f1xx_it.h"















extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;


void NMI_Handler(void)
{
  
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

void HardFault_Handler(void)
{
  
  while (1)
  {
    
  }
}

void MemManage_Handler(void)
{
  
  while (1)
  {
    
  }
}

void BusFault_Handler(void)
{
  
  while (1)
  {
    
  }
}

void UsageFault_Handler(void)
{
  
  while (1)
  {
    
  }
}

void SVC_Handler(void)
{
  
  
}

void DebugMon_Handler(void)
{
  
  
}

void PendSV_Handler(void)
{
  
  
}

void SysTick_Handler(void)
{
  
  HAL_IncTick();
  
}

void TIM1_UP_IRQHandler(void)
{
  
  HAL_TIM_IRQHandler(&htim1);
  
}

/**
  * @brief 处理 USART2 中断，交给 HAL 逐字节发送并触发发送完成回调。
  */
void USART2_IRQHandler(void)
{
  
  HAL_UART_IRQHandler(&huart2);
  
}


