/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "init.h"
#include "uart.h"
#include "onewire.h"

int main(void)
{
	uint16_t i = 0;
	onewire_port_t OWx = onewire_OW4;

	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	onewire_Init(OWx);

	init_TIM2_Configuration();

	onewire_write(OWx, onewire_low);

	uart_Configuration(UART_POLLING);

	uart_OutString("L152RE OneWire Example\r\n");

	while(1){
		uint8_t response[10];
		uint8_t OW3_Temp[11][8];
		uint8_t buffer[100];
		uint8_t j;
		uint8_t i;
		int result;

		//response = onewire_OW3_sendReset();
		response[0] = onewire_sendResetBasic(OWx);

		sprintf(buffer,"%d Received a %d\r\n", i++, response[0]);

		uart_OutString(buffer);

		result = OWFirst(OWx);
		j = 0;
		while(result){
			onewire_read_latest_ROM(&OW3_Temp[j]);
			sprintf(buffer,"%d ROM ID = %d,%d,%d,%d,%d,%d,%d,%d\r\n", i++, OW3_Temp[j][0],OW3_Temp[j][1],OW3_Temp[j][2],OW3_Temp[j][3],OW3_Temp[j][4],
					OW3_Temp[j][5],OW3_Temp[j][6],OW3_Temp[j][7]);
			uart_OutString(buffer);
			result = OWNext(OWx);
			j++;
		}
		//Trigger temperature conversion for each one
		//onewire_read_temp(OWx, OW3_Temp[0]);
		//onewire_read_temp(OWx, OW3_Temp[1]);
		//delayms(500);

		onewire_trigger_temp(OWx);
		for (i = 0; i < j; i++){
			onewire_read_stored_temp(OWx, OW3_Temp[i]);
		}
		delayms(500);

	}
}
