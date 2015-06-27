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
	spi_SPI3_Configuration();

	delayms(5000);

	flash_enable_write(0);

	uart_Configuration(UART_POLLING);

	uart_OutString("SPI RFM69HW Example Utilizing HSI Clock\r\n");

	if(sx1231h_present(SPI_RFM69_2) == 0){
		uart_OutString("SPI RFM69HW/SX1231H Failed\r\n");
	}else{
		uart_OutString("SPI RFM69HW/SX1231H Passed\r\n");

		//uart_OutString("SX1231H Configuration Before Changes\r\n");
		//sx1231h_dump_reg(SPI_RFM69_2);
		//sx1231h_init(SPI_RFM69_2);
		//delayms(100);
		//uart_OutString("SX1231H Configuration After Changes\r\n");
		//sx1231h_dump_reg(SPI_RFM69_2);
		//sx1231h_dump_select_regs();
	}

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
			//uart_OutString("SPI Flash Passed\r\n");
		}
		if(sx1231h_present(SPI_RFM69_1) == 0){
			uart_OutString("SPI RFM69HW/SX1231H Failed\r\n");
		}else{
			//uart_OutString("SPI RFM69HW/SX1231H Passed\r\n");
			sx1231h_wirelessTesting(SPI_RFM69_1, SPI_RFM69_2);
		}
		delayms(100);
	}

}
