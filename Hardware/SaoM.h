#ifndef __SAOM_H
#define __SAOM_H

#include "stm32f4xx.h"

#define RX_BUFFER_SIZE 32

extern char usart3_rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t SaoM_complete_Flag;

extern int received_number;
extern int task1, task2, task3;   // 左边三位：百位、十位、个位
extern int task4, task5, task6;   // 右边三位：百位、十位、个位
extern int text1, text2;          // 解析后的两个整数

void SaoM_Dma_Serial_Init(void);
void USART6_SendByte(uint8_t data);
void USART6_SendString(char *str);
void USART6_SendData(const uint8_t *data, uint16_t len);
void Scan_Function(void);
void ProcessReceivedExpression(void);
void Receive_QR(void);

#endif
