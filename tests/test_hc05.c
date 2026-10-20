#include "hc05.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2;
static const uint8_t *pending_data;
static uint16_t pending_length;
static unsigned int start_calls;
static HAL_StatusTypeDef start_result = HAL_OK;
static uint8_t complete_immediately;

/* 只记录 HAL 使用的缓冲区指针，不复制数据、不自动完成，模拟真实异步发送。 */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart,
                                      const uint8_t *data, uint16_t length)
{
  assert(huart == &huart2);
  ++start_calls;
  if (start_result != HAL_OK)
  {
    return start_result;
  }
  pending_data = data;
  pending_length = length;
  /* 模拟启动函数返回前就发生完成中断，验证忙标志的设置顺序。 */
  if (complete_immediately != 0U)
  {
    HAL_UART_TxCpltCallback(&huart2);
  }
  return HAL_OK;
}

/* 验证当前尚未完成的消息字节没有被调用者或下一次发送覆盖。 */
static void CheckPending(const char *expected)
{
  assert(pending_length == strlen(expected));
  assert(memcmp(pending_data, expected, pending_length) == 0);
}

/* 覆盖栈缓冲区复用、忙时跳过、长度边界、启动失败和发送完成回调。 */
int main(void)
{
  char stack_message[] = "wz=90000 mdps\r\n";
  char boundary[HC05_TX_BUFFER_SIZE + 1U];
  UART_HandleTypeDef other_uart = {0};
  unsigned int calls_before;

  assert(HC05_SendString(NULL) == HAL_ERROR);
  assert(HC05_SendString("") == HAL_OK && start_calls == 0U);
  assert(HC05_SendString(stack_message) == HAL_OK);
  assert(pending_data != (const uint8_t *)stack_message);
  memset(stack_message, 'X', sizeof(stack_message));
  CheckPending("wz=90000 mdps\r\n");

  /* 中断尚未完成时，立即返回忙，既不能再启动 HAL，也不能改写在途数据。 */
  calls_before = start_calls;
  assert(HC05_SendString("must skip") == HAL_BUSY);
  assert(start_calls == calls_before);
  CheckPending("wz=90000 mdps\r\n");
  HAL_UART_TxCpltCallback(&other_uart);
  assert(HC05_SendString("still busy") == HAL_BUSY);
  HAL_UART_TxCpltCallback(&huart2);
  assert(HC05_SendString("next") == HAL_OK);
  CheckPending("next");
  HAL_UART_TxCpltCallback(&huart2);

  memset(boundary, 'A', sizeof(boundary));
  boundary[HC05_TX_BUFFER_SIZE - 1U] = '\0';
  assert(HC05_SendString(boundary) == HAL_OK);
  assert(pending_length == HC05_TX_BUFFER_SIZE - 1U);
  HAL_UART_TxCpltCallback(&huart2);
  boundary[HC05_TX_BUFFER_SIZE - 1U] = 'A';
  boundary[HC05_TX_BUFFER_SIZE] = '\0';
  calls_before = start_calls;
  assert(HC05_SendString(boundary) == HAL_ERROR);
  assert(start_calls == calls_before);

  /* HAL 未启动传输时，模块必须释放缓冲区，后续才能重试。 */
  start_result = HAL_BUSY;
  assert(HC05_SendString("try") == HAL_BUSY);
  start_result = HAL_ERROR;
  assert(HC05_SendString("try") == HAL_ERROR);
  start_result = HAL_TIMEOUT;
  assert(HC05_SendString("try") == HAL_TIMEOUT);
  start_result = HAL_OK;
  assert(HC05_SendString("recovered") == HAL_OK);
  CheckPending("recovered");
  HAL_UART_TxCpltCallback(&huart2);

  complete_immediately = 1U;
  assert(HC05_SendString("fast") == HAL_OK);
  assert(HC05_SendString("fast again") == HAL_OK);
  puts("HC05 asynchronous TX tests passed");
  return 0;
}
