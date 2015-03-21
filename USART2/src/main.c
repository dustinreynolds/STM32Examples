/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"


int main(void)
{
  init_RCC_Configuration();

  init_GPIO_Configuration();

  uart_Configuration();

  uart_OutString("Welcome to Nucleo L152RE\r\n");

  while(1) // Don't want to exit
  {
    uint16_t Data;

    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char

    Data = USART_ReceiveData(USART2); // Collect Char

    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

    USART_SendData(USART2, Data); // Echo Char
  }
}
