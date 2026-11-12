#include "bluetooth_tune.h"
#include "hc05.h"
#include "speed_control.h"
#include "heading.h"
#include "motor.h"
#include "tof.h"
#include "maze.h"
#include "turn_control.h"
#include "tim.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim1, htim2, htim3, htim4;
UART_HandleTypeDef huart2;
static TIM_TypeDef timers[4];
static uint32_t ticks;
static uint8_t *receive_byte;
static char sent[HC05_TX_BUFFER_SIZE];
static uint8_t tx_blocked;

/* 可控节拍用于验证会话时限、心跳截止和计时回绕。 */
uint32_t HAL_GetTick(void) { return ticks; }

/* 测距桩：恒定无效，迷宫状态机启动时因感知缺失拒绝，仅需链接符号。 */
HAL_StatusTypeDef TOF_GetSensorData(TOF_Sensor sensor, TOF_Data *data)
{
  (void)sensor;
  if (data != NULL) { data->valid = 0U; }
  return HAL_ERROR;
}

/* 记录真实 HC05 发出的协议帧，立即完成或注入发送忙状态。 */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t length)
{
  assert(uart == &huart2 && length < sizeof(sent));
  if (tx_blocked != 0U) { return HAL_BUSY; }
  memcpy(sent, data, length);
  sent[length] = '\0';
  HAL_UART_TxCpltCallback(uart);
  return HAL_OK;
}

/* 保存单字节接收缓冲，模拟真实 RX 中断入口。 */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
  assert(uart == &huart2 && length == 1U);
  receive_byte = data;
  return HAL_OK;
}

/* PWM 启动桩，四路 CCR 仍由真实电机模块维护。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer; (void)channel;
  return HAL_OK;
}

/* 编码器启动桩，真实模块仍从模拟 CNT 计算速度。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer; (void)channel;
  return HAL_OK;
}

/* 提供真实编码器模块初始化回滚依赖。 */
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  (void)timer; (void)channel;
  return HAL_OK;
}

/* 注入完整命令，解析与回包均通过实际模块，不直接调用内部解析函数。 */
static void Command(const char *text)
{
  while (*text != '\0')
  {
    *receive_byte = (uint8_t)*text++;
    HAL_UART_RxCpltCallback(&huart2);
  }
  sent[0] = '\0';
  BluetoothTune_Process();
  BluetoothTune_Process();
}

/* 保持有效零速编码反馈，单独观察蓝牙心跳和任务时长保护。 */
static void Advance(uint32_t duration)
{
  uint32_t index;
  for (index = 0U; index < duration; index += 10U)
  {
    ticks += 10U;
    assert(Encoder_Update() == HAL_OK);
    (void)SpeedControl_Update();
    BluetoothTune_Process();
  }
}

/* 读取控制快照，判断协议最终实际改变了哪些控制状态。 */
static SpeedControl_Data State(void)
{
  SpeedControl_Data data;
  assert(SpeedControl_GetData(&data) == HAL_OK);
  return data;
}

/* 验证真实电机寄存器输出已经撤销，不只检查协议回执。 */
static void Stopped(void)
{
  assert(State().state == SPEED_CONTROL_IDLE);
  assert(timers[0].CCR1 == 0U && timers[0].CCR2 == 0U);
  assert(timers[1].CCR3 == 0U && timers[1].CCR4 == 0U);
}

/* 从 UART 字节流到真实 PID/电机状态，覆盖在线调参和连接故障保护。 */
int main(void)
{
  unsigned int index;
  Encoder_Data encoder_data;
  SpeedControl_Config config = {{2.5f, 1.5f, 0.0f}, {2.5f, 1.5f, 0.0f}, 1000.0f, 700};
  htim1.Instance = &timers[0]; htim2.Instance = &timers[1];
  htim3.Instance = &timers[2]; htim4.Instance = &timers[3];
  htim1.Init.Period = htim2.Init.Period = 3599U;
  htim1.Init.RepetitionCounter = 19U;
  htim3.Init.Period = htim4.Init.Period = 65535U;
  assert(Motor_Init() == HAL_OK && Encoder_Init() == HAL_OK);
  assert(SpeedControl_Init(&config) == HAL_OK && BluetoothTune_Init() == HAL_OK);
  Command("tune:1,0\n");
  assert(strcmp(sent, "cfg:1,25000,15000,0,25000,15000,0,1000,700\n") == 0);
  Command("tune:2,1,20000,3000,0,19000,4000,0,1000,65537\n");
  assert(strcmp(sent, "ack:2,1,1\n") == 0);
  assert(SpeedControl_GetConfig(&config) == HAL_OK && config.max_duty == 700);
  Command("tune:3,1,20000,3000,0,19000,4000,0,900,600\n");
  assert(strcmp(sent, "cfg:3,20000,3000,0,19000,4000,0,900,600\n") == 0);
  Command("tune:4,2,300,400,1000\n");
  assert(strcmp(sent, "ack:4,2,2\n") == 0);
  Advance(10U);
  Command("tune:5,2,300,400,1000\n");
  assert(strcmp(sent, "ack:5,2,0\n") == 0 && State().state == SPEED_CONTROL_RUNNING);
  Advance(400U);
  /* 同一事务重复到达时只回包，不重设任务开始时间。 */
  Command("tune:5,2,300,400,1000\n");
  assert(strcmp(sent, "ack:5,2,0\n") == 0);
  Command("tune:0,4\n");
  assert(sent[0] == '\0');
  Advance(600U);
  Stopped();
  Command("tune:6,2,500,500,30000\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  Advance(10U);
  assert(State().left_duty > 0);
  Command("tune:7,1,25000,15000,0,24000,14000,0,100,80\n");
  assert(State().left_target == 100.0f && State().right_target == 100.0f);
  assert(State().left_duty <= 80 && State().right_duty <= 80);
  Command("tune:8,5,-50,50\n");
  assert(strcmp(sent, "ack:8,5,0\n") == 0 && State().left_target == -50.0f);
  Command("tune:9,2,10,10,1000\n");
  assert(strcmp(sent, "ack:9,2,2\n") == 0);
  /* 即使反馈一直有效，没有心跳也必须停机，后到心跳不能恢复运动。 */
  Advance(1490U);
  Stopped();
  Command("tune:0,4\n");
  Stopped();
  Command("tune:10,2,50,50,30000\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  Command("tune:11,3\n");
  assert(strcmp(sent, "ack:11,3,0\n") == 0);
  Stopped();
  /* 发送忙期间保留配置回复，并让速度遥测让出发送机会。 */
  tx_blocked = 1U;
  Command("tune:12,0\n");
  assert(sent[0] == '\0' && !BluetoothTune_CanSendTelemetry());
  tx_blocked = 0U;
  BluetoothTune_Process();
  assert(strncmp(sent, "cfg:12,", 7U) == 0 && BluetoothTune_CanSendTelemetry());
  Command("tune:12,3\n");
  assert(strcmp(sent, "ack:12,3,1\n") == 0);
  Command("tune:13,2,50,50,30000\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  HAL_UART_ErrorCallback(&huart2);
  BluetoothTune_Process();
  BluetoothTune_Process();
  Stopped();
  /* UART 错误后先重新同步，畸形数值不得被截断后执行。 */
  Command("\ntune:14,2,50,50,30001\n");
  assert(strcmp(sent, "ack:14,2,1\n") == 0);
  Command("tune:15,2,50,50,9999999999999999999999\n");
  assert(strcmp(sent, "ack:0,0,1\n") == 0);
  Command("tune:16,0,\n");
  assert(strcmp(sent, "ack:0,0,1\n") == 0);
  /* 时钟回绕后任务时长检查仍按经过时间工作。 */
  ticks = 0xFFFFFFF0U;
  assert(Encoder_Reset() == HAL_OK);
  Advance(10U);
  Command("tune:17,2,50,50,100\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  Advance(100U);
  Stopped();
  /* 进入标定先撤销正在运行的命令，并且普通启动无法绕过模式锁。 */
  Command("tune:18,2,50,50,30000\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  Command("tune:19,6\n");
  assert(strncmp(sent, "cal:19,1,0,0,0,1,", 17U) == 0);
  Stopped();
  timers[2].CNT = (uint16_t)(timers[2].CNT - 13000U);
  timers[3].CNT = (uint16_t)(timers[3].CNT + 12000U);
  Advance(10U);
  /* 重复进入命令只回放回执，不得再次清掉已经手转得到的计数。 */
  Command("tune:19,6\n");
  assert(Encoder_GetData(&encoder_data) == HAL_OK && encoder_data.left_total == 13000);
  Command("tune:20,2,50,50,1000\n");
  assert(strcmp(sent, "ack:20,2,2\n") == 0);
  Command("tune:21,7\n");
  assert(strncmp(sent, "cal:21,1,13000,12000,1,1,", 25U) == 0);
  Stopped();
  /* 累积超过 32 位有符号范围，验证上报保留 64 位计数而不是窄化截断。 */
  for (index = 0U; index < 70000U; ++index)
  {
    ticks += 10U;
    timers[2].CNT = (uint16_t)(timers[2].CNT - 32700U);
    assert(Encoder_Update() == HAL_OK);
  }
  Command("tune:22,7\n");
  assert(strstr(sent, ",2289013000,12000,1,1,") != NULL);
  BluetoothTune_EncoderTelemetry();
  assert(strncmp(sent, "enc:1,2289013000,12000,1,1,", 26U) == 0);
  /* 计数歧义后仍回报 total_valid=0，由上位机拒绝使用不完整位移标定。 */
  ticks += 10U;
  timers[2].CNT = (uint16_t)(timers[2].CNT + 32768U);
  assert(Encoder_Update() == HAL_ERROR);
  Command("tune:23,7\n");
  assert(strstr(sent, ",2289013000,12000,0,0,") != NULL);
  Command("tune:24,6\n");
  assert(strncmp(sent, "cal:24,1,0,0,0,1,", 17U) == 0);
  Command("tune:25,3\n");
  Command("tune:26,2,50,50,1000\n");
  assert(strcmp(sent, "ack:26,2,2\n") == 0);
  Command("tune:27,8\n");
  assert(strncmp(sent, "cal:27,0,0,0,0,1,", 17U) == 0);
  Stopped();
  Advance(10U);
  Command("tune:28,7\n");
  assert(strcmp(sent, "ack:28,7,2\n") == 0);
  Command("tune:29,2,50,50,1000\n");
  assert(State().state == SPEED_CONTROL_RUNNING);
  Command("tune:30,3\n");
  Stopped();
  /* 命令 9 清零航向积分：角度归零并重建基准，不影响电机状态。 */
  {
    MPU6050_Gyro gyro = {0.0f, 0.0f, 90.0f};
    Heading_Update(&gyro, ticks);
    Heading_Update(&gyro, ticks + 100U);
    assert(Heading_GetData()->z_deg > 8.0f && Heading_GetData()->z_deg < 10.0f);
    Command("tune:31,9\n");
    assert(strcmp(sent, "ack:31,9,0\n") == 0);
    assert(Heading_GetData()->z_deg == 0.0f && Heading_GetData()->valid == 0U);
    Stopped();
    /* 运行会话中同样允许清零，电机会话不受影响。 */
    Command("tune:32,2,50,50,30000\n");
    assert(State().state == SPEED_CONTROL_RUNNING);
    Command("tune:33,9\n");
    assert(strcmp(sent, "ack:33,9,0\n") == 0);
    assert(Heading_GetData()->valid == 0U);
    Command("tune:34,3\n");
    Stopped();
  }
  /* 快捷命令必须经过真实迷宫启动条件，陀螺仪未准备好时不能放行。 */
  assert(TurnControl_Init() == HAL_OK && Maze_Init() == HAL_OK);
  Heading_Reset();
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,1\n") == 0 && Maze_IsActive() == 0U);
  {
    MPU6050_Gyro gyro = {0.0f, 0.0f, 0.0f};
    Heading_Update(&gyro, ticks);
  }
  /* 快捷命令不占用完整协议的事务号 1，也不会破坏其缓存回复。 */
  Command("tune:1,0\n");
  assert(strncmp(sent, "cfg:1,", 6U) == 0);
  Command("tune:1\r\n");
  assert(strcmp(sent, "ack:1,11,0\n") == 0 && Maze_IsActive() != 0U);
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,2\n") == 0 && Maze_IsActive() != 0U);
  Command("tune:1,0\n");
  assert(strncmp(sent, "cfg:1,", 6U) == 0);
  Command("tune:35,3\n");
  assert(Maze_IsActive() == 0U);
  Stopped();
  /* 停止后再次发送同样的快捷命令可以重新启动，不受事务去重缓存影响。 */
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,0\n") == 0 && Maze_IsActive() != 0U);
  /* 快捷启动改变了运行会话，旧停止回执不能让再次停止变成只回包不执行。 */
  Command("tune:35,3\n");
  assert(strcmp(sent, "ack:35,3,0\n") == 0 && Maze_IsActive() == 0U);
  Stopped();
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,0\n") == 0 && Maze_IsActive() != 0U);
  ticks += 300U;
  Maze_Process();
  /* 测距桩无效使迷宫故障锁存，快捷命令不能暗中清故障并重启。 */
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,2\n") == 0 && Maze_IsActive() == 0U);
  Command("tune:36,3\n");
  /* 标定锁仍生效，短命令不能绕过完整协议的模式互斥。 */
  Command("tune:37,6\n");
  Command("tune:1\n");
  assert(strcmp(sent, "ack:1,11,2\n") == 0 && Maze_IsActive() == 0U);
  Command("tune:38,8\n");
  Command("tune:01\n");
  assert(strcmp(sent, "ack:0,0,1\n") == 0);
  Stopped();

  puts("Bluetooth UART-to-motor tuning integration tests passed");
  return 0;
}
