/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "stdio.h"
#include "init.h"
#include "uart.h"
#include "spi.h"
#include "flash.h"


int main(void)
{
	uint8_t result;
	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	init_TIM2_Configuration();

	spi_SPI2_Configuration();

	flash_enable_write(0);

	uart_Configuration(UART_POLLING);

	uart_OutString("SPI Flash Example Utilizing HSI Clock\r\n");

	if (flash_present()){
		result = flash_stress_test();

		if (result == 0){
			uart_OutString("SPI Stress Test Failed\r\n");
		}else{
			uart_OutString("SPI Stress Test Passed\r\n");
		}
	}

	while(1){

		if (result == 0){
			uart_OutString("SPI Stress Test Failed\r\n");
		}else{
			uart_OutString("SPI Stress Test Passed\r\n");
		}

		delayms(1000);
	}

}
