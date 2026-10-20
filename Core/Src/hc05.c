#include "hc05.h"
#include "usart.h"
#include <string.h>

/* 发送期间由 HAL 中断读取，生命周期不能依赖调用者的栈缓冲区。 */
static uint8_t tx_buffer[HC05_TX_BUFFER_SIZE];
/* 主循环置忙，发送完成中断清忙；先置忙再启动，避免完成回调与置忙竞争。 */
static volatile uint8_t tx_busy;

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
