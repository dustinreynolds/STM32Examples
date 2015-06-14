/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 *
 * Copyright (c) 2015, Dustin Reynolds
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of [project] nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

	i2c_ds3231_reset();

	i2c_init();

	i2c_ds3231_init(I2C2);

	uart_Configuration(USARTx, UART_INTERRUPT_RX);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");
	uart_OutString(USARTx," Example for interfacing with a DS3231 RTC\r\n");

	{
		uint8_t control;
		i2c_ds3231_write_reg(I2C2,DS3231_CONTROL_REG, DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN);
		i2c_ds3231_read_register(I2C2,DS3231_CONTROL_REG, &control);

		if (control != (DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN)){
			uart_OutString(USARTx,"DS3231 Configured incorrectly!\r\n");
		}else{
			uart_OutString(USARTx,"DS3231 Present\r\n");
		}
	}

	if (i2c_ds3231_test_rtc(I2C2) == 0){
		DS3231_time_t time1;
		DS3231_date_t date1;
		uart_OutString(USARTx,"DS3231 RTC Functioning as expected.\r\n");
		time1.seconds = 0;
		time1.minutes = 57;
		time1.hours = 11;
		time1.is_ampm_time = 0;
		time1.am_pm = 0;
		date1.day = 1;
		date1.date = 14;
		date1.month = 6;
		date1.year = 15;
		i2c_ds3231_set_time_date(I2C2, time1, date1);
	}else{
		uart_OutString(USARTx,"DS3231 RTC NOT functioning as expected.\r\n");
		i2c_ds3231_test_rtc(I2C2);
	}



	while(1){
		i2c_ds3231_read_registers(I2C2,0x00, data, 19);

		for (i = 0; i < 19; i++){
			char buffer[30];
			sprintf(buffer, "%02x:%02x,", i, data[i]);
			uart_OutString(USARTx, buffer);
		}
		uart_OutString(USARTx,"\r\n");
	}

}
