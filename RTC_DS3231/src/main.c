/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"
#include "i2c.h"

int main(void)
{
	uint8_t data[20];
	uint8_t i;
	USART_TypeDef * USARTx = USART2;
	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	init_TIM2_Configuration();

	i2c_init();



	uart_Configuration(USARTx, UART_INTERRUPT_RX);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");
	uart_OutString(USARTx," Example for interfacing with a DS3231 RTC\r\n");

	while(1){
		uint8_t control;
		char buffer[30];
		i2c_ds3231_read_control_register(I2C2, &control);
		sprintf(buffer, "%02x:%02x,", DS3231_CONTROL_REG, control);
		uart_OutString(USARTx, buffer);
		delayms(100);

		i2c_ds3231_write_control_register(I2C2,DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
	}


//	while(1){
//		i2c_ds3231_dump_all_reg(I2C2, data);
//
//		for (i = 0; i < 19; i++){
//			char buffer[30];
//			sprintf(buffer, "%02x:%02x,", i, data[i]);
//			uart_OutString(USARTx, buffer);
//		}
//		uart_OutString(USARTx,"\r\n");
//		//delayms(500);
//	}

// Test Write and read for single registers
//This code works
//	while(1){
//		uint8_t control;
//		char buffer[30];
//		i2c_ds3231_read_control_register(I2C2, &control);
//		sprintf(buffer, "%02x:%02x,", DS3231_CONTROL_REG, control);
//		uart_OutString(USARTx, buffer);
//		delayms(100);
//
//		i2c_ds3231_write_control_register(I2C2,DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
//	}



}
