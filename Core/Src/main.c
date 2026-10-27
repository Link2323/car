#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "speed_control.h"
#include "hc05.h"
#include "bluetooth_tune.h"
#include <stdio.h>
/* USER CODE END Includes */



/* USER CODE BEGIN PD */
/* 蓝牙调参模式优先，禁止旧的自动试转流程与电脑命令同时控制电机。 */
#ifndef BLUETOOTH_TUNE_ENABLE
#define BLUETOOTH_TUNE_ENABLE 1
#endif
/* 架空并确认编码器方向后才可开启试转；默认上电只采集，不驱动车轮。 */
#ifndef MOTOR_DEBUG_SPEED_ENABLE
#define MOTOR_DEBUG_SPEED_ENABLE 0
#endif
#define MOTOR_DEBUG_SPEED_COUNTS_PER_S 500.0f
#define MOTOR_DEBUG_SPEED_DURATION_MS 4000U
/* 200 ms 一帧，避免 9600 波特率下调试信息占满发送缓冲。 */
#ifndef SPEED_DEBUG_TELEMETRY_ENABLE
#define SPEED_DEBUG_TELEMETRY_ENABLE 1
#endif
#define SPEED_DEBUG_TELEMETRY_PERIOD_MS 200U
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
/* 保存双轮增量、累计计数和计数率，供控制逻辑及调试器读取。 */
Encoder_Data car_encoder;
HAL_StatusTypeDef encoder_status = HAL_ERROR;
/* 保存三路测距快照和逐路状态；car_tof 保留前方兼容快照。 */
TOF_Data car_tof;
HAL_StatusTypeDef tof_status = HAL_ERROR;
TOF_Data car_tof_sensors[TOF_SENSOR_COUNT];
HAL_StatusTypeDef tof_sensor_status[TOF_SENSOR_COUNT];
/* 已按保存的轮端标定完成落地 50/75/100 mm/s 短试转，100 mm/s 复测两次。
   速度反馈仅 10 ms 计数率、遥测仅 5 Hz，不将本组视为最终最优参数。 */
static const SpeedControl_Config speed_config = {
  {0.8f, 1.5f, 0.0f}, {0.6f, 1.5f, 0.0f}, 2100.0f, 700
};
SpeedControl_Data car_speed;
HAL_StatusTypeDef speed_control_status = HAL_ERROR;
/* USER CODE END PV */

void SystemClock_Config(void);


/* USER CODE BEGIN 0 */
/* 可选一次性试转：获得有效反馈后启动，定时结束或故障后不自动重试。 */
static void Speed_DebugProcess(void)
{
#if MOTOR_DEBUG_SPEED_ENABLE && !BLUETOOTH_TUNE_ENABLE
  static uint8_t phase = 0U;
  static uint32_t started = 0U;
  HAL_StatusTypeDef status;
  if (phase == 0U)
  {
    status = SpeedControl_Enable();
    if (status == HAL_OK)
    {
      started = HAL_GetTick();
      status = SpeedControl_SetTarget(MOTOR_DEBUG_SPEED_COUNTS_PER_S,
                                     MOTOR_DEBUG_SPEED_COUNTS_PER_S);
      phase = (status == HAL_OK) ? 1U : 2U;
    }
    else if (status != HAL_BUSY)
    {
      phase = 2U;
    }
  }
  else if (phase == 1U)
  {
    (void)SpeedControl_GetData(&car_speed);
    if (car_speed.state == SPEED_CONTROL_FAULT)
    {
      /* 保留故障锁存供调试，不在此处清除或自动恢复。 */
      phase = 2U;
    }
    else if ((uint32_t)(HAL_GetTick() - started) >= MOTOR_DEBUG_SPEED_DURATION_MS)
    {
      SpeedControl_Stop();
      phase = 2U;
    }
  }
#endif
}

/* 发送整数 FireWater 帧；目标/实测取整为 counts/s，占空比单位千分比。
   只监测不驱动时也发布编码器实测，便于手转确认两轮方向。 */
static void Speed_DebugTelemetry(void)
{
#if SPEED_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < SPEED_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  last_tick = now;
  length = snprintf(frame, sizeof(frame), "speed:%d,%d,%ld,%ld,%ld,%ld,%d,%d\n",
                    (int)car_speed.state, (int)car_speed.fault,
                    (long)car_speed.left_target, (long)car_speed.right_target,
                    (long)car_encoder.left_counts_per_s,
                    (long)car_encoder.right_counts_per_s,
                    (int)car_speed.left_duty, (int)car_speed.right_duty);
  /* 串口忙时丢弃该帧，不等待发送完成，不影响控制周期。 */
  if (length > 0 && (unsigned int)length < sizeof(frame))
  {
    (void)HC05_SendString(frame);
  }
#endif
}

/* 统一刷新三路测距快照，同时更新前方兼容变量。 */
static void TOF_RefreshSnapshots(void)
{
  unsigned int i;
  /* 每路有效性与错误独立保存，单路失效时仍可观察另外两路。 */
  for (i = 0U; i < TOF_SENSOR_COUNT; ++i)
  {
    (void)TOF_GetSensorData((TOF_Sensor)i, &car_tof_sensors[i]);
    tof_sensor_status[i] = TOF_GetSensorStatus((TOF_Sensor)i);
  }
  car_tof = car_tof_sensors[TOF_FRONT];
}

/* USER CODE END 0 */

/**
  * @brief  初始化小车外设和传感器，在主循环采集测距、陀螺仪和编码器数据。
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
     此时电机 PWM 尚未启动；失败时保留错误状态供调试器查看。 */
  mpu6050_init_status = MPU6050_Init();
  mpu6050_status = mpu6050_init_status;
  if (mpu6050_status == HAL_OK)
  {
    mpu6050_cal_status = MPU6050_CalibrateGyro(200U);
    mpu6050_status = mpu6050_cal_status;
  }
  /* 依次释放三路 XSHUT 并改址；单路失败仍继续采集其他路。 */
  tof_status = TOF_Init();
  TOF_RefreshSnapshots();

  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* 初始化后保持电机禁用，等待控制逻辑显式使能。 */
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
  speed_control_status = SpeedControl_Init(&speed_config);
  if (speed_control_status != HAL_OK)
  {
    Error_Handler();
  }
  /* 启动蓝牙接收，不自动使能电机；调参命令仅在主循环解析。 */
#if BLUETOOTH_TUNE_ENABLE
  if (BluetoothTune_Init() != HAL_OK) { Error_Handler(); }
#endif

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* 先检查上一轮耗时，防止长时间阻塞后继续使用旧占空比。 */
    speed_control_status = SpeedControl_Update();
#if BLUETOOTH_TUNE_ENABLE
    BluetoothTune_Process();
#endif
    /* 电机状态机与速度计算分离，持续服务启动等待和换向空档。 */
    Motor_Process();
    /* 主循环查询三路已完成的测量，不等待传感器完成一次激光测距。 */
    {
      HAL_StatusTypeDef update_status = TOF_Update();
      if (update_status != HAL_BUSY)
      {
        tof_status = update_status;
      }
      TOF_RefreshSnapshots();
    }
    /* I2C 返回后先检查旧反馈年龄，再采样，避免新时间戳掩盖超时。 */
    speed_control_status = SpeedControl_Update();
    {
      HAL_StatusTypeDef update_status = Encoder_Update();
      /* 只有到达采样周期才更新调试数据；无新采样时保留上一份快照。 */
      if (update_status != HAL_BUSY)
      {
        encoder_status = update_status;
        (void)Encoder_GetData(&car_encoder);
      }
    }
    /* 有新编码器采样才计算 PID，同一份数据不会重复积分。 */
    speed_control_status = SpeedControl_Update();
    {
      static uint32_t last_read_tick = 0;
      uint32_t now = HAL_GetTick();

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
    }
    /* 陀螺仪读取也可能阻塞，返回后先处理控制保护，再执行调试操作。 */
    speed_control_status = SpeedControl_Update();
    Speed_DebugProcess();
#if BLUETOOTH_TUNE_ENABLE
    BluetoothTune_Process();
#endif
    (void)SpeedControl_GetData(&car_speed);
    Speed_DebugTelemetry();
#if BLUETOOTH_TUNE_ENABLE
    /* 累计计数与原速度帧分开传输，保留速度曲线的原有 200 ms 周期。 */
    BluetoothTune_EncoderTelemetry();
#endif

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
