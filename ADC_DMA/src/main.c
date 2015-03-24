/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include <stdio.h>
#include "stdbool.h"
#include "stm32l1xx.h"
#include "init.h"
#include "uart.h"
#include "adc.h"

extern bool flag_ADCDMA_TransferComplete;

int main(void)
{
	uint32_t deviceID;
	deviceID = DBGMCU_GetDEVID();
	float coreTemp;
	char buffer[20];

	init_RCC_Configuration();

	init_GPIO_Configuration();

	/* Set internal voltage regulator to 1.8V */
	PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

	/* Wait Until the Voltage Regulator is ready */
	while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;

	adc_configureDMA();

	adc_init();

	adc_acquireTemperatureData();

	uart_Configuration(UART_POLLING);

	uart_OutString("Welcome to Nucleo L152RE\r\n");

	while(1){

		if (flag_ADCDMA_TransferComplete){
			adc_processTempData();
			coreTemp = adc_getCalTemp();
			snprintf(buffer, sizeof(buffer), "Temp %f\n\n", coreTemp);
			uart_OutString(buffer);

			//start next acquire session.
			adc_acquireTemperatureData();
		}

		uart_OutString("Switching to interrupt mode\r\n");

		uart_switch_mode(UART_INTERRUPT_RX);
		uint32_t i = 0;
		for (i = 0; i<8000000; i++);  //roughly 10 second delay

		uart_OutString("Switching to polling mode\r\n");

		uart_switch_mode(UART_POLLING);

		for (i = 0; i < 800000; i++){
			uint16_t Data;

			/*Only get a character if it is ready*/
			if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET){

				Data = USART_ReceiveData(USART2); // Collect Char

				while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

				USART_SendData(USART2, Data); // Echo Char
			}
		}
	}
}
