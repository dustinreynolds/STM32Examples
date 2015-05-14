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
#include "sx1231h.h"


int main(void)
{
	uint8_t result = 0;
	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	init_TIM2_Configuration();

	spi_SPI2_Configuration();

	delayms(20);

	flash_enable_write(0);

	uart_Configuration(UART_POLLING);

	uart_OutString("SPI RFM69HW Example Utilizing HSI Clock\r\n");

	if (flash_present()){
		//result = flash_stress_test();

		if (result == 0){
			uart_OutString("SPI Stress Test Failed\r\n");
		}else{
			uart_OutString("SPI Stress Test Passed\r\n");
		}
	}

	while(1){

		if (flash_present() == 0){
			uart_OutString("SPI Flash Failed\r\n");
		}else{
			uart_OutString("SPI Flash Passed\r\n");
		}
		if(sx1231h_present() == 0){
			uart_OutString("SPI RFM69HW/SX1231H Failed\r\n");
		}else{
			uart_OutString("SPI RFM69HW/SX1231H Passed\r\n");
		}
		delayms(100);
	}

}
