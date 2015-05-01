/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"
#include "onewire.h"

int main(void)
{
	uint16_t i = 0;

	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	onewire_OW3Init();

	onewire_TIM2_Configuration();

	onewire_OW3Write(onewire_low);

	uart_Configuration(UART_POLLING);

	uart_OutString("L152RE OneWire Example\r\n");

	while(1){
		uint8_t response;
		uint8_t buffer[50];
		//response = onewire_OW3_sendReset();
		response = onewire_OW3_sendResetBasic();

		sprintf(buffer,"%d Received a %d\r\n", i++, response);

		uart_OutString(buffer);

		delayms(1000);
	}
}
