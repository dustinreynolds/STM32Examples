/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"

__IO uint8_t showtime[50] = {0};
__IO uint8_t showdate[50] = {0};

extern RTC_DateTypeDef RTC_DateStructure;
extern RTC_TimeTypeDef RTC_TimeStructure;

int main(void)
{
	uint32_t i;
	init_RCC_Configuration();

	init_GPIO_Configuration();

	RTC_Config();

	uart_Configuration(UART_INTERRUPT_RX);

	uart_OutString("Welcome to Nucleo L152RE\r\n");

	while(1){

		/* Get the RTC current Time */
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
		/* Get the RTC current Date */
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		/* Display time Format : hh:mm:ss */
		sprintf((char*)showtime,"%.2d:%.2d:%.2d",RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);
		/* Display date Format : mm-dd-yy */
		sprintf((char*)showdate,"%.2d-%.2d-%.2d",RTC_DateStructure.RTC_Month, RTC_DateStructure.RTC_Date, 2000 + RTC_DateStructure.RTC_Year);

		uart_OutString("Time:");
		uart_OutString(showtime);
		uart_OutString("  Date: ");
		uart_OutString(showdate);
		uart_OutString("\r\n");

		for (i = 0; i<800000; i++);  //roughly 10 second delay
	}
}
