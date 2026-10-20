

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"





/* USER CODE BEGIN EC */
extern volatile uint32_t timer_1ms_ticks;
/* USER CODE END EC */



void Error_Handler(void);



#define BT_TX_Pin GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define TOF_L_XSHUT_Pin GPIO_PIN_4
#define TOF_L_XSHUT_GPIO_Port GPIOA
#define TOF_F_XSHUT_Pin GPIO_PIN_5
#define TOF_F_XSHUT_GPIO_Port GPIOA
#define ENC_L_A_Pin GPIO_PIN_6
#define ENC_L_A_GPIO_Port GPIOA
#define ENC_L_B_Pin GPIO_PIN_7
#define ENC_L_B_GPIO_Port GPIOA
#define ML_BI_Pin GPIO_PIN_10
#define ML_BI_GPIO_Port GPIOB
#define ML_FI_Pin GPIO_PIN_11
#define ML_FI_GPIO_Port GPIOB
#define MR_BI_Pin GPIO_PIN_8
#define MR_BI_GPIO_Port GPIOA
#define MR_FI_Pin GPIO_PIN_9
#define MR_FI_GPIO_Port GPIOA
#define TOF_R_XSHUT_Pin GPIO_PIN_5
#define TOF_R_XSHUT_GPIO_Port GPIOB
#define ENC_R_A_Pin GPIO_PIN_6
#define ENC_R_A_GPIO_Port GPIOB
#define ENC_R_B_Pin GPIO_PIN_7
#define ENC_R_B_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB



#ifdef __cplusplus
}
#endif

#endif 
