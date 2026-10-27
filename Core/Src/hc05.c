#include "hc05.h"
#include "usart.h"
#include <string.h>

/* 发送期间由 HAL 中断读取，生命周期不能依赖调用者的栈缓冲区。 */
static uint8_t tx_buffer[HC05_TX_BUFFER_SIZE];
/* 主循环置忙，发送完成中断清忙；先置忙再启动，避免完成回调与置忙竞争。 */
static volatile uint8_t tx_busy;
/* 中断只入队字节，主循环只出队；保留一个空位区分满与空。 */
#define HC05_RX_SIZE 256U
static uint8_t rx_byte;
static uint8_t rx_buffer[HC05_RX_SIZE];
static volatile uint16_t rx_write;
static volatile uint16_t rx_read;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_armed;
static char rx_line[HC05_TX_BUFFER_SIZE];
static uint16_t rx_length;
static uint8_t rx_discard;

/* 启动一次接收，不清空在途缓冲，避免重试时拼接出错误命令。 */
HAL_StatusTypeDef HC05_InitReceive(void)
{
  HAL_StatusTypeDef status;
  if (rx_armed != 0U) { return HAL_OK; }
  status = HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  if (status == HAL_OK) { rx_armed = 1U; }
  return status;
}

/* 在主循环恢复未处于接收状态的 UART，不做等待。 */
void HC05_ProcessReceive(void)
{
  if (rx_armed == 0U) { (void)HC05_InitReceive(); }
}

/* 中断接收单字节并立即挂起下一字节，禁止在回调中解析或驱动电机。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next;
  if (huart != &huart2) { return; }
  next = (uint16_t)((rx_write + 1U) % HC05_RX_SIZE);
  if (next == rx_read) { rx_overflow = 1U; }
  else
  {
    rx_buffer[rx_write] = rx_byte;
    rx_write = next;
  }
  rx_armed = 0U;
  (void)HC05_InitReceive();
}

/* UART 错误后整行作废；主循环负责重试接收，绝不执行错误行后缀。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    rx_overflow = 1U;
    rx_armed = 0U;
  }
}

/* 逐字节组帧；过长、丢字节或非法字符均丢弃整行，防止部分命令生效。 */
HAL_StatusTypeDef HC05_ReadLine(char *line, uint16_t capacity)
{
  uint8_t byte;
  if (line == NULL || capacity < HC05_TX_BUFFER_SIZE) { return HAL_ERROR; }
  if (rx_overflow != 0U)
  {
    rx_read = rx_write;
    rx_overflow = 0U;
    rx_length = 0U;
    rx_discard = 1U;
    return HAL_ERROR;
  }
  while (rx_read != rx_write)
  {
    byte = rx_buffer[rx_read];
    rx_read = (uint16_t)((rx_read + 1U) % HC05_RX_SIZE);
    if (byte == '\n')
    {
      if (rx_discard != 0U) { rx_discard = 0U; rx_length = 0U; continue; }
      rx_line[rx_length] = '\0';
      memcpy(line, rx_line, rx_length + 1U);
      rx_length = 0U;
      return HAL_OK;
    }
    if (rx_discard != 0U || byte == '\r') { continue; }
    if (byte < 32U || byte > 126U || rx_length >= HC05_TX_BUFFER_SIZE - 1U)
    {
      rx_length = 0U;
      rx_discard = 1U;
      return HAL_ERROR;
    }
    rx_line[rx_length++] = (char)byte;
  }
  return HAL_BUSY;
}

/* 复制并启动一条异步消息；忙时立即返回，不排队、不等待、不覆盖在途数据。 */
HAL_StatusTypeDef HC05_SendString(const char *message)
{
  uint16_t length = 0U;
  HAL_StatusTypeDef status;

  if (message == NULL)
  {
    return HAL_ERROR;
  }

  /* 只允许主循环生产消息，中断仅释放缓冲区，因此无需长时间关中断。 */
  if (tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  /* 有界检查消息长度，拒绝超长字符串，避免静默截断诊断信息。 */
  while (length < HC05_TX_BUFFER_SIZE && message[length] != '\0')
  {
    ++length;
  }
  if (length >= HC05_TX_BUFFER_SIZE)
  {
    return HAL_ERROR;
  }
  if (length == 0U)
  {
    return HAL_OK;
  }

  memcpy(tx_buffer, message, length);
  tx_busy = 1U;
  status = HAL_UART_Transmit_IT(&huart2, tx_buffer, length);
  /* 未成功启动时不会有完成回调，必须立即释放忙状态供下一次尝试。 */
  if (status != HAL_OK)
  {
    tx_busy = 0U;
  }
  return status;
}

/* HAL 在最后一字节完成、发送状态恢复 READY 后回调，才允许复用缓冲区。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* 其他串口的完成事件不能提前释放 USART2 正在使用的缓冲区。 */
  if (huart == &huart2)
  {
    tx_busy = 0U;
  }
}
