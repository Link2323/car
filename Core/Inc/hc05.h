#ifndef HC05_H
#define HC05_H

#include "main.h"

/* 内部缓冲区含结束符，单条消息最多发送 127 字节。 */
#define HC05_TX_BUFFER_SIZE 128U

/* 通过 USART2 中断发送字符串，不自动添加换行符，仅允许在主循环调用。
   消息会复制到内部缓冲区，返回后调用者可立即修改或释放原字符串。
   HAL_OK 表示已启动发送（空字符串无需发送），不代表线路发送完成；
   HAL_BUSY 表示上一条尚未完成，本次消息未接收，调用者可跳过本次调试消息；
   NULL 或超长消息返回 HAL_ERROR，不截断。USART2 发送由本模块独占。 */
HAL_StatusTypeDef HC05_SendString(const char *message);

#endif
