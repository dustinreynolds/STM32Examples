/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"


int main(void)
{
	USART_TypeDef * USARTx = USART2;
	init_RCC_Configuration();

	init_GPIO_Configuration();

	uart_Configuration(USARTx, UART_POLLING);

	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	while(1){

		uart_OutString(USARTx,"Switching to interrupt mode\r\n");

		uart_switch_mode(USARTx, UART_INTERRUPT_RX);
		uint32_t i = 0;
		for (i = 0; i<80000000; i++);  //roughly 10 second delay

		uart_OutString(USARTx,"Switching to polling mode\r\n");

		uart_switch_mode(USARTx,UART_POLLING);

		for (i = 0; i < 8000000; i++){
			uint16_t Data;

			/*Only get a character if it is ready*/
			if(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) != RESET){

				Data = USART_ReceiveData(USARTx); // Collect Char

				while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait for Empty

				USART_SendData(USARTx, Data); // Echo Char
			}
		}
	}
}
