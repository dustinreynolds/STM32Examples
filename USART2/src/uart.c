/*
 * uart.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"

void uart_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE);
}

void uart_OutString(char *s)
{
  while(*s)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

    USART_SendData(USART2, *s++); // Send Char
  }
}


