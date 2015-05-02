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
		uint8_t response[10];
		uint8_t buffer[100];
		uint8_t j;
		//response = onewire_OW3_sendReset();
		response[0] = onewire_OW3_sendResetBasic();

		sprintf(buffer,"%d Received a %d\r\n", i++, response[0]);

		uart_OutString(buffer);

		//delayms(100);

		//onewire_OW3_ReadBasic();
		//onewire_OW3_sendByte(0xFF);
		onewire_OW3_sendByte(0xCC); //skip rom
		onewire_OW3_sendByte(0xBE); //read scratchpad
		for (j = 0; j < 9; j++){
			response[j] = onewire_OW3_readByte();
		}
		sprintf(buffer,"%d readbyte = %d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", i++, response[0],response[1],response[2],response[3],response[4],
		                                                                    response[5],response[6],response[7],response[8]);
		//80,5,85,0,127,255,12,16,33
		uart_OutString(buffer);
		delayms(500);
		response[0] = onewire_OW3_sendResetBasic();
		onewire_OW3_sendByte(0x33); //read rom, only for 1 device on bus
		for (j = 0; j < 8; j++){
			response[j] = onewire_OW3_readByte();
		}
		sprintf(buffer,"%d ROM ID = %d,%d,%d,%d,%d,%d,%d,%d\r\n", i++, response[0],response[1],response[2],response[3],response[4],
				                                                                    response[5],response[6],response[7]);
		//40,255,193,161,108,20,3,170
		uart_OutString(buffer);
		delayms(500);
	}
}
