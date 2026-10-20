#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "hc05.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "tof.h"
/* USER CODE END Includes */



/* USER CODE BEGIN PD */
/* 临时电机调试开关：设为 0 后恢复默认禁用、不自动转动的行为。 */
#define MOTOR_DEBUG_ENABLE       1U
#define MOTOR_DEBUG_WAIT_MS      3000U
#define MOTOR_DEBUG_RAMP_MS      2000U
#define MOTOR_DEBUG_HOLD_MS      3000U
#define MOTOR_DEBUG_MAX_DUTY     500U
/* USER CODE END PD */



/* USER CODE BEGIN PV */
volatile uint32_t timer_1ms_ticks = 0;
MPU6050_Gyro car_gyro = {0.0f, 0.0f, 0.0f};
HAL_StatusTypeDef mpu6050_status = HAL_ERROR;
/* 分别保存各阶段结果，避免统一的无效消息掩盖实际失败位置。 */
HAL_StatusTypeDef mpu6050_init_status = HAL_ERROR;
HAL_StatusTypeDef mpu6050_cal_status = HAL_ERROR;
HAL_StatusTypeDef mpu6050_read_status = HAL_ERROR;
uint32_t gyro_last_update_tick = 0;
uint8_t gyro_valid = 0;
/* ST-Link 可观察双轮增量、累计计数和计数率，蓝牙输出计数率曲线。 */
Encoder_Data car_encoder;
HAL_StatusTypeDef encoder_status = HAL_ERROR;
/* 前方测距快照和初始化状态，便于通过调试器核查故障。 */
TOF_Data car_tof;
HAL_StatusTypeDef tof_status = HAL_ERROR;
#if MOTOR_DEBUG_ENABLE
/* 从外设初始化完成后计时，先给调试和供电操作预留静止等待。 */
static uint32_t motor_debug_start_tick;
#endif
/* USER CODE END PV */

void SystemClock_Config(void);


/* USER CODE BEGIN 0 */
#if MOTOR_DEBUG_ENABLE
/* 循环调试：等待 3 秒，2 秒内升至 50%，保持 3 秒后停机并进入下一轮等待。
   没有轮速闭环，低占空比不保证能起转，也不代表恒定低转速。
   全程不阻塞；接口出错则停止并锁定测试，不继续自动重试。 */
static void Motor_DebugProcess(void)
{
  static uint8_t started = 0U;
  static uint8_t faulted = 0U;
  static int16_t last_duty = -1;
  uint32_t elapsed = (uint32_t)(HAL_GetTick() - motor_debug_start_tick);
  uint32_t ramp_elapsed;
  int16_t duty;

  /* 每轮等待阶段不使能电机；故障后保持停机，直到重新复位。 */
  if (faulted != 0U || elapsed < MOTOR_DEBUG_WAIT_MS)
  {
    return;
  }
  /* 主循环因其他任务延误时，超过软件计时时长后先停机，不补跑旧步骤。
     调试器暂停可能同时停止系统节拍，因此此处不是独立的硬件超时保护。 */
  if (elapsed >= MOTOR_DEBUG_WAIT_MS + MOTOR_DEBUG_RAMP_MS + MOTOR_DEBUG_HOLD_MS)
  {
    Motor_Disable();
    /* 清除上一轮状态，从当前时刻重新等待 3 秒，避免跨轮保持旧的 PWM。 */
    started = 0U;
    last_duty = -1;
    motor_debug_start_tick = HAL_GetTick();
    return;
  }
  if (started == 0U)
  {
    if (Motor_Enable() != HAL_OK)
    {
      Motor_Disable();
      faulted = 1U;
      return;
    }
    started = 1U;
  }

  ramp_elapsed = elapsed - MOTOR_DEBUG_WAIT_MS;
  /* 斜坡结束后保持目标上限，避免占空比继续增加。 */
  if (ramp_elapsed > MOTOR_DEBUG_RAMP_MS)
  {
    ramp_elapsed = MOTOR_DEBUG_RAMP_MS;
  }
  duty = (int16_t)(MOTOR_DEBUG_MAX_DUTY * ramp_elapsed / MOTOR_DEBUG_RAMP_MS);
  if (duty != last_duty)
  {
    /* 两轮使用前进正命令，驱动层已按实测对右轮方向进行反转校正。 */
    if (Motor_SetDuty(duty, duty) != HAL_OK)
    {
      Motor_Disable();
      faulted = 1U;
      return;
    }
    last_duty = duty;
  }
}
#endif
/* USER CODE END 0 */

/**
  * @brief  初始化小车外设和传感器，在主循环采集前方距离并通过蓝牙输出。
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
	
  /* USER CODE BEGIN 2 */
  /* 上电保持小车静止：先初始化陀螺仪，再采集约 1 秒的静止零偏。
     此时电机 PWM 尚未启动；失败时保留错误状态供蓝牙和调试器查看。 */
  mpu6050_init_status = MPU6050_Init();
  mpu6050_status = mpu6050_init_status;
  if (mpu6050_status == HAL_OK)
  {
    mpu6050_cal_status = MPU6050_CalibrateGyro(200U);
    mpu6050_status = mpu6050_cal_status;
  }
  /* 只释放前方 XSHUT；失败也继续运行，通过蓝牙状态通道报告。 */
  tof_status = TOF_Init();
  (void)TOF_GetData(&car_tof);
  /* 串口只输出 FireWater 数值帧，不混入启动文字。 */

  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* 电机模块先保持禁用，调试开关打开时由循环测试流程延时使能。 */
  if (Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 编码器模块负责启动 TIM3/TIM4 并建立软件计数零点。 */
  encoder_status = Encoder_Init();
  if (encoder_status != HAL_OK)
  {
    Error_Handler();
  }

#if MOTOR_DEBUG_ENABLE
  motor_debug_start_tick = HAL_GetTick();
#endif
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 先处理测试的超时停机，再服务电机状态机，独立于传感器是否在线。 */
#if MOTOR_DEBUG_ENABLE
    Motor_DebugProcess();
#endif
    Motor_Process();
    /* 主循环查询已完成的前方测量，不等待传感器完成一次激光测距。 */
    {
      HAL_StatusTypeDef update_status = TOF_Update();
      if (update_status != HAL_BUSY)
      {
        tof_status = update_status;
      }
      (void)TOF_GetData(&car_tof);
    }
    {
      HAL_StatusTypeDef update_status = Encoder_Update();
      /* 只有到达采样周期才更新调试数据；无新采样时保留上一份快照。 */
      if (update_status != HAL_BUSY)
      {
        encoder_status = update_status;
        (void)Encoder_GetData(&car_encoder);
      }
    }
    {
      static uint32_t last_send_tick = 0;
      static uint32_t last_read_tick = 0;
      uint32_t now = HAL_GetTick();
      char message[96];

      /* 每 5 ms 尝试读取一份新样本，阻塞式 I2C 始终在主循环执行。
         初始化或校准失败时不发布未经校准的角速度。 */
      if (mpu6050_status == HAL_OK && (uint32_t)(now - last_read_tick) >= 5U)
      {
        last_read_tick = now;
        mpu6050_read_status = MPU6050_ReadGyro(&car_gyro);
        if (mpu6050_read_status == HAL_OK)
        {
          gyro_last_update_tick = HAL_GetTick();
          gyro_valid = 1U;
        }
        else if (mpu6050_read_status != HAL_BUSY)
        {
          /* 通信失败立即使数据失效；后续周期继续尝试读取。 */
          gyro_valid = 0U;
        }
      }

      /* 蓝牙每 100 ms 输出前方距离与诊断通道，缺测时也发帧，便于定位接线问题。 */
      now = HAL_GetTick();
      if ((uint32_t)(now - last_send_tick) >= 100U)
      {
        last_send_tick = now;
        (void)TOF_GetData(&car_tof);
        /* FireWater CSV：距离(mm)、有效位、诊断状态、器件原始状态。
           无效距离使用 -1，避免绘图或上位机把旧数据当成当前障碍物距离。 */
        (void)snprintf(message, sizeof(message), "tof:%ld,%u,%u,%u\n",
                       car_tof.valid != 0U ? (long)car_tof.distance_mm : -1L,
                       (unsigned int)car_tof.valid, (unsigned int)car_tof.state,
                       (unsigned int)car_tof.raw_status);
        /* 中断发送会复制局部消息；忙时丢弃当前帧，继续服务传感器。 */
        (void)HC05_SendString(message);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置系统时钟，使主频达到 72 MHz。
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* TIM1 提供 1 ms 软件节拍；耗时的传感器读取和串口发送留在主循环。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 只累计 TIM1 的更新事件，防止其他定时器改变该节拍。 */
  if (htim->Instance == TIM1)
  {
    timer_1ms_ticks++;
    
  }
}
/* USER CODE END 4 */

/**
  * @brief 发生初始化等致命错误时，先禁用电机，再停留等待调试。
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 尚未初始化时禁用接口为空操作；已经启动时清零输出并取消待执行命令。 */
  Motor_Disable();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  
}
#endif 
