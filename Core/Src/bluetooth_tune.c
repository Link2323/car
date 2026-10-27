#include "bluetooth_tune.h"
#include "hc05.h"
#include "speed_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define TUNE_HEARTBEAT_TIMEOUT_MS 1500U
#define TUNE_MAX_RUN_MS 30000U
#define TUNE_GAIN_SCALE 10000.0f

static char reply[HC05_TX_BUFFER_SIZE];
static uint8_t reply_pending;
static uint8_t session_running;
static uint32_t heartbeat_tick;
static uint32_t started_tick;
static uint32_t duration_ms;
/* 缓存最近事务，重复命令只回放响应，不重新启动电机或延长试转。 */
static char last_command[HC05_TX_BUFFER_SIZE];
static char last_reply[HC05_TX_BUFFER_SIZE];
static long last_id;
static uint8_t calibration_active;
static uint32_t encoder_telemetry_tick;

/* 将有符号 64 位累计计数转换为十进制，兼容 ARMCC5 与主机编译器。
   INT64_MIN 先加一再取反，避免对最小负数直接求负的溢出。 */
static void BluetoothTune_TotalText(int64_t value, char *output)
{
  char reverse[20];
  unsigned int count = 0U;
  unsigned int index = 0U;
  uint64_t magnitude = (value < 0) ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
  do
  {
    reverse[count++] = (char)('0' + magnitude % 10U);
    magnitude /= 10U;
  } while (magnitude != 0U);
  if (value < 0) { output[index++] = '-'; }
  /* 倒序输出数字，零计数也会生成一位，不发送空字段。 */
  while (count != 0U) { output[index++] = reverse[--count]; }
  output[index] = '\0';
}

/* 共用编码快照格式；标定回复含事务号，周期帧不含事务号。 */
static HAL_StatusTypeDef BluetoothTune_FormatEncoder(char *buffer, long id)
{
  Encoder_Data data;
  char left[21];
  char right[21];
  int length;
  if (Encoder_GetData(&data) != HAL_OK) { return HAL_ERROR; }
  BluetoothTune_TotalText(data.left_total, left);
  BluetoothTune_TotalText(data.right_total, right);
  if (id > 0L)
  {
    length = snprintf(buffer, HC05_TX_BUFFER_SIZE, "cal:%ld,%u,%s,%s,%u,%u,%lu\n", id,
      (unsigned int)calibration_active, left, right, (unsigned int)data.sample_valid,
      (unsigned int)data.total_valid, (unsigned long)data.timestamp_ms);
  }
  else
  {
    length = snprintf(buffer, HC05_TX_BUFFER_SIZE, "enc:%u,%s,%s,%u,%u,%lu\n",
      (unsigned int)calibration_active, left, right, (unsigned int)data.sample_valid,
      (unsigned int)data.total_valid, (unsigned long)data.timestamp_ms);
  }
  return (length > 0 && length < HC05_TX_BUFFER_SIZE) ? HAL_OK : HAL_ERROR;
}

/* 解析严格十进制有符号整数，限定字段个数，拒绝溢出、空字段和多余字符。 */
static int BluetoothTune_Parse(const char *text, long *values, int capacity)
{
  int count = 0;
  char *end;
  if (strncmp(text, "tune:", 5U) != 0) { return -1; }
  text += 5;
  while (*text != '\0' && count < capacity)
  {
    if (!((*text >= '0' && *text <= '9') || *text == '-')) { return -1; }
    errno = 0;
    values[count] = strtol(text, &end, 10);
    if (end == text || errno == ERANGE || values[count] < -2147483647L ||
        values[count] > 2147483647L) { return -1; }
    ++count;
    if (*end == '\0') { return count; }
    if (*end != ',' || end[1] == '\0') { return -1; }
    text = end + 1;
  }
  return -1;
}

/* 回复只包含数值，沿用 FireWater 风格，不在串口中混入调试文字。 */
static void BluetoothTune_Ack(long id, long operation, HAL_StatusTypeDef status)
{
  (void)snprintf(reply, sizeof(reply), "ack:%ld,%ld,%d\n", id, operation, (int)status);
  reply_pending = 1U;
}

/* 以万分之一增益精度回读配置，避免固件使用浮点 printf。 */
static void BluetoothTune_ConfigReply(long id)
{
  SpeedControl_Config config;
  if (SpeedControl_GetConfig(&config) != HAL_OK)
  {
    BluetoothTune_Ack(id, 0L, HAL_ERROR);
    return;
  }
  (void)snprintf(reply, sizeof(reply), "cfg:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d\n", id,
    (long)(config.left.kp * TUNE_GAIN_SCALE + 0.5f),
    (long)(config.left.ki * TUNE_GAIN_SCALE + 0.5f),
    (long)(config.left.kd * TUNE_GAIN_SCALE + 0.5f),
    (long)(config.right.kp * TUNE_GAIN_SCALE + 0.5f),
    (long)(config.right.ki * TUNE_GAIN_SCALE + 0.5f),
    (long)(config.right.kd * TUNE_GAIN_SCALE + 0.5f),
    (long)config.max_target_counts_per_s, (int)config.max_duty);
  reply_pending = 1U;
}

/* 解码并应用完整参数包，成功后回读实际配置，失败时保留原配置。 */
static void BluetoothTune_Configure(long id, const long *values)
{
  SpeedControl_Config config;
  HAL_StatusTypeDef status;
  unsigned int index;
  for (index = 2U; index < 8U; ++index)
  {
    if (values[index] < 0L || values[index] > 10000000L)
    {
      BluetoothTune_Ack(id, 1L, HAL_ERROR);
      return;
    }
  }
  if (values[8] < 1L || values[8] > 1000000L || values[9] < 1L || values[9] > 1000L)
  {
    BluetoothTune_Ack(id, 1L, HAL_ERROR);
    return;
  }
  config.left.kp = (float)values[2] / TUNE_GAIN_SCALE;
  config.left.ki = (float)values[3] / TUNE_GAIN_SCALE;
  config.left.kd = (float)values[4] / TUNE_GAIN_SCALE;
  config.right.kp = (float)values[5] / TUNE_GAIN_SCALE;
  config.right.ki = (float)values[6] / TUNE_GAIN_SCALE;
  config.right.kd = (float)values[7] / TUNE_GAIN_SCALE;
  config.max_target_counts_per_s = (float)values[8];
  config.max_duty = (int16_t)values[9];
  status = SpeedControl_Configure(&config);
  if (status == HAL_OK) { BluetoothTune_ConfigReply(id); }
  else { BluetoothTune_Ack(id, 1L, status); }
}

/* 一次性启动必须从待机进入；重复启动不能解除故障或延长已运行任务。 */
static HAL_StatusTypeDef BluetoothTune_Start(const long *values)
{
  SpeedControl_Data data;
  HAL_StatusTypeDef status;
  if (calibration_active != 0U) { return HAL_BUSY; }
  if (values[4] < 100L || values[4] > TUNE_MAX_RUN_MS ||
      values[2] < -1000000L || values[2] > 1000000L ||
      values[3] < -1000000L || values[3] > 1000000L) { return HAL_ERROR; }
  if (SpeedControl_GetData(&data) != HAL_OK || data.state != SPEED_CONTROL_IDLE)
  {
    return HAL_BUSY;
  }
  status = SpeedControl_Enable();
  if (status != HAL_OK) { return status; }
  status = SpeedControl_SetTarget((float)values[2], (float)values[3]);
  if (status == HAL_OK)
  {
    session_running = 1U;
    heartbeat_tick = started_tick = HAL_GetTick();
    duration_ms = (uint32_t)values[4];
  }
  return status;
}

/* 命令分派：0~5 为调参，6 进入标定并清零，7 读取计数，8 退出标定。 */
static void BluetoothTune_Handle(const char *line)
{
  long values[10];
  int count = BluetoothTune_Parse(line, values, 10);
  HAL_StatusTypeDef status = HAL_ERROR;
  if (count < 2 || values[0] < 0L)
  {
    BluetoothTune_Ack(0L, 0L, HAL_ERROR);
    return;
  }
  /* 心跳不占用事务号，也不回包，降低 9600 波特率下的带宽占用。 */
  if (values[0] == 0L && values[1] == 4L && count == 2)
  {
    heartbeat_tick = HAL_GetTick();
    return;
  }
  if (values[0] == 0L) { BluetoothTune_Ack(0L, values[1], HAL_ERROR); return; }
  if (values[0] == last_id)
  {
    if (strcmp(line, last_command) == 0)
    {
      (void)strcpy(reply, last_reply);
      reply_pending = 1U;
    }
    else { BluetoothTune_Ack(values[0], values[1], HAL_ERROR); }
    return;
  }
  if (values[1] == 0L && count == 2) { BluetoothTune_ConfigReply(values[0]); }
  else if (values[1] == 1L && count == 10) { BluetoothTune_Configure(values[0], values); }
  else if (values[1] >= 6L && values[1] <= 8L && count == 2)
  {
    if (values[1] == 6L)
    {
      /* 即便正在试转，也先禁用电机再重建软件零点；标定绝不自动驱动。 */
      session_running = 0U;
      SpeedControl_Stop();
      calibration_active = 1U;
      status = Encoder_Reset();
    }
    else if (values[1] == 8L)
    {
      session_running = 0U;
      SpeedControl_Stop();
      calibration_active = 0U;
      status = HAL_OK;
    }
    else { status = (calibration_active != 0U) ? HAL_OK : HAL_BUSY; }
    if (status == HAL_OK && BluetoothTune_FormatEncoder(reply, values[0]) == HAL_OK)
    {
      reply_pending = 1U;
    }
    else { BluetoothTune_Ack(values[0], values[1], (status == HAL_OK) ? HAL_ERROR : status); }
  }
  else
  {
    if (values[1] == 2L && count == 5) { status = BluetoothTune_Start(values); }
    else if (values[1] == 3L && count == 2)
    {
      session_running = 0U;
      SpeedControl_Stop();
      status = HAL_OK;
    }
    else if (values[1] == 5L && count == 4 && session_running != 0U &&
             values[2] >= -1000000L && values[2] <= 1000000L &&
             values[3] >= -1000000L && values[3] <= 1000000L)
    {
      status = SpeedControl_SetTarget((float)values[2], (float)values[3]);
    }
    BluetoothTune_Ack(values[0], values[1], status);
  }
  last_id = values[0];
  (void)strcpy(last_command, line);
  (void)strcpy(last_reply, reply);
}

/* 调试运行仅由显式启动命令触发，不在连接或上电时使能电机。 */
HAL_StatusTypeDef BluetoothTune_Init(void)
{
  return HC05_InitReceive();
}

/* 保护先于命令解析，迟到的心跳不能挽救已经超时的运行会话。 */
void BluetoothTune_Process(void)
{
  char line[HC05_TX_BUFFER_SIZE];
  HAL_StatusTypeDef status;
  SpeedControl_Data data;
  uint32_t now = HAL_GetTick();
  HC05_ProcessReceive();
  /* 模式锁存在断线后继续有效，重连或普通停止都不能隐式重新允许运动。 */
  if (calibration_active != 0U) { SpeedControl_Stop(); }
  if (session_running != 0U)
  {
    if (SpeedControl_GetData(&data) != HAL_OK || data.state != SPEED_CONTROL_RUNNING)
    {
      session_running = 0U;
    }
    else if ((uint32_t)(now - heartbeat_tick) >= TUNE_HEARTBEAT_TIMEOUT_MS ||
             (uint32_t)(now - started_tick) >= duration_ms)
    {
      SpeedControl_Stop();
      session_running = 0U;
    }
  }
  if (reply_pending != 0U)
  {
    if (HC05_SendString(reply) == HAL_OK) { reply_pending = 0U; }
    else { return; }
  }
  status = HC05_ReadLine(line, sizeof(line));
  if (status == HAL_OK && line[0] != '\0') { BluetoothTune_Handle(line); }
  else if (status == HAL_ERROR)
  {
    /* 丢包或缓冲溢出时撤销调试运动，不在未知命令流上继续运行。 */
    if (session_running != 0U) { SpeedControl_Stop(); session_running = 0U; }
    BluetoothTune_Ack(0L, 0L, HAL_ERROR);
  }
}

/* 回复尚未启动发送时，周期帧让出串口；已启动后由 HC05 忙标志保护。 */
uint8_t BluetoothTune_CanSendTelemetry(void)
{
  return (uint8_t)(reply_pending == 0U);
}

/* 周期上报累计轮程，正常运行低频发送，避免占用原速度曲线的带宽。 */
void BluetoothTune_EncoderTelemetry(void)
{
  char frame[HC05_TX_BUFFER_SIZE];
  uint32_t now = HAL_GetTick();
  uint32_t period = (calibration_active != 0U) ? 200U : 1000U;
  if (reply_pending != 0U || (uint32_t)(now - encoder_telemetry_tick) < period) { return; }
  if (BluetoothTune_FormatEncoder(frame, 0L) == HAL_OK && HC05_SendString(frame) == HAL_OK)
  {
    encoder_telemetry_tick = now;
  }
}
