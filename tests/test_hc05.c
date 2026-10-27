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
static uint8_t *rx_destination;
static HAL_StatusTypeDef receive_result = HAL_OK;

/* 模拟单字节中断接收，保存缓冲地址供测试注入真实回调。 */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
  assert(huart == &huart2 && length == 1U);
  if (receive_result == HAL_OK) { rx_destination = data; }
  return receive_result;
}

/* 逐字节模拟 UART 中断，保留生产者与主循环解析的实际模块边界。 */
static void Receive(const char *text)
{
  while (*text != '\0')
  {
    assert(rx_destination != NULL);
    *rx_destination = (uint8_t)*text++;
    HAL_UART_RxCpltCallback(&huart2);
  }
}

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
  char line[HC05_TX_BUFFER_SIZE];
  unsigned int index;
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
  /* 接收启动失败允许主循环重试，不影响原有中断发送功能。 */
  receive_result = HAL_ERROR;
  assert(HC05_InitReceive() == HAL_ERROR);
  receive_result = HAL_OK;
  HC05_ProcessReceive();
  assert(rx_destination != NULL);
  assert(HC05_InitReceive() == HAL_OK);
  assert(HC05_ReadLine(NULL, sizeof(line)) == HAL_ERROR);
  assert(HC05_ReadLine(line, 4U) == HAL_ERROR);
  Receive("tune:1,0");
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_BUSY);
  Receive("\r\ntune:2,3\n");
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_OK && strcmp(line, "tune:1,0") == 0);
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_OK && strcmp(line, "tune:2,3") == 0);
  /* 超长行必须丢弃至换行，不能把尾部拼成新的启动命令。 */
  for (index = 0; index < 140U; ++index) { Receive("x"); }
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_ERROR);
  Receive("tune:3,2,100,100,3000\ntune:4,0\n");
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_OK && strcmp(line, "tune:4,0") == 0);
  /* 环形队列溢出后丢弃残留并重新同步，恢复后仍可接受完整帧。 */
  for (index = 0; index < 300U; ++index) { Receive("x"); }
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_ERROR);
  Receive("suffix\ntune:5,0\n");
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_OK && strcmp(line, "tune:5,0") == 0);
  HAL_UART_ErrorCallback(&huart2);
  HC05_ProcessReceive();
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_ERROR);
  Receive("\ntune:6,0\n");
  assert(HC05_ReadLine(line, sizeof(line)) == HAL_OK && strcmp(line, "tune:6,0") == 0);
  puts("HC05 asynchronous TX tests passed");
  return 0;
}
