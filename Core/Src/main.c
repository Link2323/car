#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "heading.h"
#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "speed_control.h"
#include "turn_control.h"
#include "maze.h"
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
/* 200 ms 一帧，避免 9600 波特率下调试信息占满发送缓冲。
   当前仅观察航向积分，速度/MPU 诊断/轮程累计帧默认关闭，需要时再打开。 */
#ifndef SPEED_DEBUG_TELEMETRY_ENABLE
#define SPEED_DEBUG_TELEMETRY_ENABLE 0
#endif
#define SPEED_DEBUG_TELEMETRY_PERIOD_MS 200U
/* 航向积分调试帧：输出 z 轴航偏角，手持旋转或试车时观察积分与漂移。 */
#ifndef HEADING_DEBUG_TELEMETRY_ENABLE
#define HEADING_DEBUG_TELEMETRY_ENABLE 1
#endif
#define HEADING_DEBUG_TELEMETRY_PERIOD_MS 200U
/* MPU 上电诊断帧：暴露初始化/校准/读取结果和地址探测，定位 head 全零的原因。 */
#ifndef MPU_DEBUG_TELEMETRY_ENABLE
#define MPU_DEBUG_TELEMETRY_ENABLE 1
#endif
#define MPU_DEBUG_TELEMETRY_PERIOD_MS 1000U
/* 轮程累计帧开关与航向帧独立；蓝牙调参模式下默认不发送。 */
#ifndef ENCODER_TELEMETRY_ENABLE
#define ENCODER_TELEMETRY_ENABLE 0
#endif
/* 转向状态诊断帧：调试期观察转向状态机与目标收敛过程。 */
#ifndef TURN_DEBUG_TELEMETRY_ENABLE
#define TURN_DEBUG_TELEMETRY_ENABLE 1
#endif
#define TURN_DEBUG_TELEMETRY_PERIOD_MS 1000U
/* 迷宫探索诊断帧：状态机、三路距离与决策，实跑时观察感知与节奏。 */
#ifndef MAZE_DEBUG_TELEMETRY_ENABLE
#define MAZE_DEBUG_TELEMETRY_ENABLE 1
#endif
#define MAZE_DEBUG_TELEMETRY_PERIOD_MS 1000U
/* 三路测距实时帧：补偿值与原始值并列，用于偏移标定与读数核对。 */
#ifndef TOF_DEBUG_TELEMETRY_ENABLE
#define TOF_DEBUG_TELEMETRY_ENABLE 1
#endif
#define TOF_DEBUG_TELEMETRY_PERIOD_MS 1000U
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
   速度反馈仅 10 ms 计数率、遥测仅 5 Hz，不将本组视为最终最优参数。
   2026-10-27：占空比上限 700 提高到 850——7.4 V 电池下原地转向
   摩擦扭矩大，70% 输出带不动单轮（架空正常、落地不转），85% 实测
   左右 90 度转向恢复到位（误差约 2 度）。 */
static const SpeedControl_Config speed_config = {
  {0.8f, 1.5f, 0.0f}, {0.6f, 1.5f, 0.0f}, 2100.0f, 850
};
SpeedControl_Data car_speed;
HAL_StatusTypeDef speed_control_status = HAL_ERROR;
/* 陀螺仪闭环转向状态快照，供调试器与后续遥测观察。 */
TurnControl_Data car_turn;
HAL_StatusTypeDef turn_control_status = HAL_ERROR;
/* 左手法则迷宫探索状态快照。 */
Maze_Data car_maze;
HAL_StatusTypeDef maze_status = HAL_ERROR;
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

/* 浮点值按两位小数格式化为十进制文本，返回写入字符数。
   串口帧不使用固件不支持的 %f 格式，改用整数运算拆分符号、整数与小数。 */
static int Heading_FormatFloat(char *buffer, unsigned int size, float value)
{
  long scaled = (long)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
  const char *sign = (scaled < 0L) ? "-" : "";
  long magnitude = (scaled < 0L) ? -scaled : scaled;
  return snprintf(buffer, size, "%s%ld.%02ld", sign,
                  magnitude / 100L, magnitude % 100L);
}

/* 发送航向 FireWater 帧：数据有效、积分有效、z 轴积分角度（航偏角，度）。
   角度直接输出两位小数的真实值，串口工具无需换算即可绘图；
   发送失败不推进节拍，下一轮主循环重试。 */
static void Heading_DebugTelemetry(void)
{
#if HEADING_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  const Heading_Data *heading;
  char frame[HC05_TX_BUFFER_SIZE];
  int offset;
  if ((uint32_t)(now - last_tick) < HEADING_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  heading = Heading_GetData();
  offset = snprintf(frame, sizeof(frame), "head:%u,%u,",
                    (unsigned int)gyro_valid, (unsigned int)heading->valid);
  if (offset > 0 && (unsigned int)offset + 12U < sizeof(frame))
  {
    offset += Heading_FormatFloat(frame + (unsigned int)offset,
                                  sizeof(frame) - (unsigned int)offset,
                                  heading->z_deg);
    if ((unsigned int)offset + 1U < sizeof(frame))
    {
      frame[offset++] = '\n';
      frame[offset] = '\0';
      if (HC05_SendString(frame) == HAL_OK)
      {
        last_tick = now;
      }
    }
  }
#endif
}

/* 发送 MPU 上电诊断帧：初始化、零偏校准、最近读取结果和两个候选地址的身份探测。
   HAL 状态 0 成功、1 错误、2 忙、3 超时；读取持续为 2 表示数据就绪从不置位，
   身份字段非 0x68/0x70 表示总线应答的是不支持的其他芯片。 */
static void MPU_DebugTelemetry(void)
{
#if MPU_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  const MPU6050_ProbeInfo *probe;
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < MPU_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  probe = MPU6050_GetProbeInfo();
  length = snprintf(frame, sizeof(frame), "mpu:%d,%d,%d,%d,%u,%d,%u\n",
                    (int)mpu6050_init_status, (int)mpu6050_cal_status,
                    (int)mpu6050_read_status,
                    (int)probe->status[0], (unsigned int)probe->who_am_i[0],
                    (int)probe->status[1], (unsigned int)probe->who_am_i[1]);
  if (length > 0 && (unsigned int)length < sizeof(frame) &&
      HC05_SendString(frame) == HAL_OK)
  {
    last_tick = now;
  }
#endif
}

/* 发送转向状态诊断帧：状态机、目标、误差、差速请求与运行时长。
   状态 0 待机、1 运行、2 完成、3 故障；角度与速度放大 100 倍取整。 */
static void Turn_DebugTelemetry(void)
{
#if TURN_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < TURN_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  length = snprintf(frame, sizeof(frame), "turn:%d,%ld,%ld,%ld,%lu\n",
                    (int)car_turn.state,
                    (long)(car_turn.target_deg * 100.0f +
                           (car_turn.target_deg >= 0.0f ? 0.5f : -0.5f)),
                    (long)(car_turn.error_deg * 100.0f +
                           (car_turn.error_deg >= 0.0f ? 0.5f : -0.5f)),
                    (long)(car_turn.speed_counts_per_s * 100.0f +
                           (car_turn.speed_counts_per_s >= 0.0f ? 0.5f : -0.5f)),
                    (unsigned long)car_turn.elapsed_ms);
  if (length > 0 && (unsigned int)length < sizeof(frame) &&
      HC05_SendString(frame) == HAL_OK)
  {
    last_tick = now;
  }
#endif
}

/* 发送迷宫探索诊断帧：状态、决策、三路距离、感知有效位、航向基准
   与节奏统计。状态 0 待机、1 决策、2 转向、3 直行、4 左口滑行、
   5 故障；决策 0 无、1 左、2 直、3 右、4 掉头；距离单位 mm，
   航向放大 100 倍。 */
static void Maze_DebugTelemetry(void)
{
#if MAZE_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < MAZE_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  length = snprintf(frame, sizeof(frame), "maze:%d,%d,%u,%u,%u,%u,%ld,%lu,%lu,%u\n",
                    (int)car_maze.state, (int)car_maze.decision,
                    (unsigned int)car_maze.left_mm,
                    (unsigned int)car_maze.front_mm,
                    (unsigned int)car_maze.right_mm,
                    (unsigned int)car_maze.sensor_valid,
                    (long)(car_maze.heading_target_deg * 100.0f +
                           (car_maze.heading_target_deg >= 0.0f ? 0.5f : -0.5f)),
                    (unsigned long)car_maze.drive_elapsed_ms,
                    (unsigned long)car_maze.steps,
                    (unsigned int)car_maze.fault_reason);
  if (length > 0 && (unsigned int)length < sizeof(frame) &&
      HC05_SendString(frame) == HAL_OK)
  {
    last_tick = now;
  }
#endif
}

/* 发送三路测距实时帧：前/左/右各两个距离字段（补偿后、原始值），
   最后为有效位掩码（1 前、2 左、4 右）；距离单位 mm。
   原始值不做偏移补偿，供左右路标定与前方 -32mm 补偿核对。 */
static void TOF_DebugTelemetry(void)
{
#if TOF_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < TOF_DEBUG_TELEMETRY_PERIOD_MS ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  length = snprintf(frame, sizeof(frame), "tof:%u,%u,%u,%u,%u,%u,%u\n",
                    (unsigned int)car_tof_sensors[TOF_FRONT].distance_mm,
                    (unsigned int)car_tof_sensors[TOF_FRONT].raw_distance_mm,
                    (unsigned int)car_tof_sensors[TOF_LEFT].distance_mm,
                    (unsigned int)car_tof_sensors[TOF_LEFT].raw_distance_mm,
                    (unsigned int)car_tof_sensors[TOF_RIGHT].distance_mm,
                    (unsigned int)car_tof_sensors[TOF_RIGHT].raw_distance_mm,
                    (unsigned int)((car_tof_sensors[TOF_FRONT].valid != 0U ? 1U : 0U) |
                                    (car_tof_sensors[TOF_LEFT].valid != 0U ? 2U : 0U) |
                                    (car_tof_sensors[TOF_RIGHT].valid != 0U ? 4U : 0U)));
  if (length > 0 && (unsigned int)length < sizeof(frame) &&
      HC05_SendString(frame) == HAL_OK)
  {
    last_tick = now;
  }
#endif
}

/* 独立诊断帧按前、左、右输出状态/HAL状态/原始状态码，区分回波无效与掉线。
   忙时下轮再试，保留原 tof 七字段格式，避免影响已有串口工具。 */
static void TOF_StatusTelemetry(void)
{
#if TOF_DEBUG_TELEMETRY_ENABLE
  static uint32_t last_tick = 0U;
  uint32_t now = HAL_GetTick();
  char frame[HC05_TX_BUFFER_SIZE];
  int length;
  if ((uint32_t)(now - last_tick) < 1000U ||
      !BluetoothTune_CanSendTelemetry()) { return; }
  length = snprintf(frame, sizeof(frame), "tofdiag:%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
    (unsigned int)car_tof_sensors[0].state, (unsigned int)tof_sensor_status[0],
    (unsigned int)car_tof_sensors[0].raw_status,
    (unsigned int)car_tof_sensors[1].state, (unsigned int)tof_sensor_status[1],
    (unsigned int)car_tof_sensors[1].raw_status,
    (unsigned int)car_tof_sensors[2].state, (unsigned int)tof_sensor_status[2],
    (unsigned int)car_tof_sensors[2].raw_status);
  if (length > 0 && (unsigned int)length < sizeof(frame) &&
      HC05_SendString(frame) == HAL_OK)
  {
    last_tick = now;
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
    /* 零偏校准完成后角度从零开始；后续由串口命令 9 随时清零。 */
    if (mpu6050_status == HAL_OK)
    {
      Heading_Reset();
    }
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
  turn_control_status = TurnControl_Init();
  if (turn_control_status != HAL_OK)
  {
    Error_Handler();
  }
  maze_status = Maze_Init();
  if (maze_status != HAL_OK)
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
      static uint32_t last_retry_tick = 0;
      static uint32_t busy_since_tick = 0;
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
          busy_since_tick = 0U;
          /* 新样本立即积分；时间戳取读取完成时刻，期间的主循环阻塞由真实 dt 体现。 */
          Heading_Update(&car_gyro, gyro_last_update_tick);
        }
        else if (mpu6050_read_status != HAL_BUSY)
        {
          /* 通信失败立即使数据失效；后续周期继续尝试读取。 */
          gyro_valid = 0U;
          busy_since_tick = 0U;
        }
        else if (busy_since_tick == 0U)
        {
          busy_since_tick = now;
        }
        else if ((uint32_t)(now - busy_since_tick) >= 1000U)
        {
          /* 持续 BUSY 说明数据就绪从不置位（芯片内部状态挂死），初始化
             成功后同样会发生；置失败态走下方重试路径，重新初始化自愈。 */
          busy_since_tick = 0U;
          gyro_valid = 0U;
          mpu6050_status = HAL_ERROR;
        }
      }
      /* 初始化或校准失败后周期性重试：电池上电瞬间的电源抖动可能让
         芯片卡在不应答地址的状态，电源稳定后的重新初始化通常可唤醒。
         校准含约 1 秒阻塞，仅在速度环待机且无转向时执行，避免运动
         控制被阻塞；重试成功后清旧数据并重建积分基准。 */
      else if (mpu6050_status != HAL_OK &&
               (uint32_t)(now - last_retry_tick) >= 2000U &&
               car_speed.state == SPEED_CONTROL_IDLE &&
               TurnControl_IsActive() == 0U)
      {
        last_retry_tick = now;
        mpu6050_init_status = MPU6050_Init();
        mpu6050_status = mpu6050_init_status;
        if (mpu6050_status == HAL_OK)
        {
          gyro_valid = 0U;
          mpu6050_cal_status = MPU6050_CalibrateGyro(200U);
          mpu6050_status = mpu6050_cal_status;
          if (mpu6050_status == HAL_OK)
          {
            Heading_Reset();
          }
        }
      }
    }
    /* 陀螺仪读取也可能阻塞，返回后先处理控制保护，再执行调试操作。 */
    speed_control_status = SpeedControl_Update();
    /* z 角度闭环转向：基于刚刷新的航偏角更新差速目标，含到位/超时保护。 */
    TurnControl_Process();
    (void)TurnControl_GetData(&car_turn);
    /* 左手法则迷宫探索状态机：感知决策、调度转向与直行段。 */
    Maze_Process();
    (void)Maze_GetData(&car_maze);
    Speed_DebugProcess();
#if BLUETOOTH_TUNE_ENABLE
    BluetoothTune_Process();
#endif
    (void)SpeedControl_GetData(&car_speed);
    /* 测距初始化可能阻塞，只在速度环待机、无寻路或转向任务时恢复故障路。
       寻路故障仍保持锁存，传感器恢复后也必须显式停止清障并重新启动。 */
    (void)TOF_Recover((uint8_t)(car_speed.state == SPEED_CONTROL_IDLE &&
                              Maze_IsActive() == 0U &&
                              TurnControl_IsActive() == 0U));
    Speed_DebugTelemetry();
    Heading_DebugTelemetry();
    MPU_DebugTelemetry();
    Turn_DebugTelemetry();
    Maze_DebugTelemetry();
    TOF_DebugTelemetry();
    TOF_StatusTelemetry();
#if BLUETOOTH_TUNE_ENABLE && ENCODER_TELEMETRY_ENABLE
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
