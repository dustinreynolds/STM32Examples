/*
 * init.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */

#include "stm32l1xx.h"


void init_RCC_Configuration(void)
{
	/* --------------------------- System Clocks Configuration -----------------*/
	/* USART2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable APB1 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
}

void init_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PF.4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure the GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void init_HSI(void){
	//http://blog.tkjelectronics.dk/2010/02/stm32-internal-8mhz-clock-setup-routine/
	FLASH_SetLatency(FLASH_Latency_1);

	RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_6, RCC_PLLDiv_3);//32 MHz
	RCC_PLLCmd(ENABLE);

	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
}



void init_TIM2_Configuration(void){
	TIM_TimeBaseInitTypeDef timerInitStructure;
	NVIC_InitTypeDef nvicStructure;

	//Period between interrupts is (Period-1)/(32000000/(Prescaler-1))
	timerInitStructure.TIM_Prescaler = 31;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 100; //results in 10us on, 10us off
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

}

void init_TIM2_Change_Period(uint32_t period){
	//By disabling events, we can modify the Period register safely.
	//Once updated, re-enabling UEV events clears the existing counters,
	//giving us a clean slate to start from.
	TIM2->CR1 |= TIM_CR1_UDIS; //Disable UEV events
	TIM2->ARR = period;
	TIM2->CR1 &= ~TIM_CR1_UDIS; //Enable UEV events
}
