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
	long i;
	init_RCC_Configuration();

	init_GPIO_Configuration();

	uart_Configuration(UART_POLLING);

	uart_OutString("Simple Program to verify Pin connections\r\n");

	while(1){

		uart_OutString("All High\r\n");
		set_all_high();
		for (i = 0; i<800000; i++);  //roughly 10 second delay
		uart_OutString("All Low\r\n");
		set_all_low();
		for (i = 0; i<800000; i++);  //roughly 10 second delay
	}
}
