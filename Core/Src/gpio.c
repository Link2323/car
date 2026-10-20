

#include "gpio.h"





void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, TOF_L_XSHUT_Pin|TOF_F_XSHUT_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(TOF_R_XSHUT_GPIO_Port, TOF_R_XSHUT_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = TOF_L_XSHUT_Pin|TOF_F_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TOF_R_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_R_XSHUT_GPIO_Port, &GPIO_InitStruct);

}


