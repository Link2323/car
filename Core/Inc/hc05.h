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

/* USART2 初始化后启动单字节中断接收，所有解析和电机操作留在主循环。 */
HAL_StatusTypeDef HC05_InitReceive(void);
/* 主循环调用，尝试恢复 UART 错误或启动失败后的接收。 */
void HC05_ProcessReceive(void);
/* 从中断接收环形缓冲取一条完整行，去掉 CR/LF；容量需为 128 字节。
   无完整行返回 HAL_BUSY；溢出或过长行返回 HAL_ERROR 并丢弃至换行，
   不允许把截断命令或尾部碎片当成新命令执行。 */
HAL_StatusTypeDef HC05_ReadLine(char *line, uint16_t capacity);

#endif
