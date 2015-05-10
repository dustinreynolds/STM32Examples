/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "string.h"
#include "init.h"
#include "uart.h"
#include "spi.h"


int main(void)
{
	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	init_TIM2_Configuration();

	spi_SPI2_Configuration();

	uart_Configuration(UART_POLLING);

	uart_OutString("SPI Flash Example Utilizing HSI Clock\r\n");

	while(1){
		uint8_t response[10];
		uint8_t buffer[100];

		GPIOC->BSRRH |= GPIO_Pin_8;
		response[0] = spi_send(0x90);
		response[1] = spi_send(0x00);
		response[2] = spi_send(0x00);
		response[3] = spi_send(0x00);
		response[4] = spi_send(0x00);  //responds MISO 0x01
		response[5] = spi_send(0x00);  //responds MISO 0x17
		GPIOC->BSRRL |= GPIO_Pin_8;

		sprintf(buffer,"Flash: %d %d %d %d %d %d\r\n",response[0], response[1], response[2], response[3], response[4], response[5]);
		uart_OutString(buffer);

		delayms(100);
	}
}
