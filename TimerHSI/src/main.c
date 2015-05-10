/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"

volatile static TIM2_flag = 0;
void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM2_flag = 1;
    }
}

void delayms(uint32_t msec){
	while (msec-- > 0){
		delayus(1000);
		//asm("nop");
	}
}

void delayus(uint32_t usec){
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2_flag = 0;
	init_TIM2_Change_Period(usec);
	while(TIM2_flag == 0){
		//__WFI();  //can cause debugger to think it has disconnected, explore http://nuttx.org/doku.php?id=wiki:howtos:jtag-debugging
	};
	TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}

int main(void)
{
	init_HSI();

	init_RCC_Configuration();

	init_GPIO_Configuration();

	init_TIM2_Configuration();

	uart_Configuration(UART_POLLING);

	while(1){
		//uart_OutString("Welcome to Nucleo L152RE\r\n");
		delayus(100);
		GPIOC->BSRRH = GPIO_Pin_6;
		//uart_OutString("1Welcome to Nucleo L152RE\r\n");
		delayus(250);
		GPIOC->BSRRL = GPIO_Pin_6;
		uart_OutString("2Welcome to Nucleo L152RE\r\n");

		delayms(1000);
	}
}
