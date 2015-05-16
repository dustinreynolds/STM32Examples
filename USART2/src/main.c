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

	uart_Configuration(USARTx, UART_INTERRUPT_RX);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	uart_Configuration(UART5, UART_INTERRUPT_RX);

	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	while(1){
		//Let GPS send us stuff in the background
	}
}
