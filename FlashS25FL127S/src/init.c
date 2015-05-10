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

	/* TIM 2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* SPI Pins */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* Flash Chip Select */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
}

void init_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ----------------------------*/
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Initialize GPIO for SPI, making sure all CS pins are set to output high */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

	/* CS for Flash */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->BSRRL |= GPIO_Pin_8;

	/* CS for RF Radio just in case it's present */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRL |= GPIO_Pin_12;
}

void init_HSI(void){
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
	timerInitStructure.TIM_Period = 1000; //results in 10us on, 10us off
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

volatile static TIM2_flag = 0;
void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM2_flag = 1;
	}
}

void __inline__ init_TIM2_Change_Period(uint16_t period){
	//By disabling events, we can modify the Period register safely.
	//Once updated, re-enabling UEV events clears the existing counters,
	//giving us a clean slate to start from.
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_UDIS; //Disable UEV events
	TIM2_flag = 0;
	TIM2->ARR = period;
	TIM2->CR1 &= ~TIM_CR1_UDIS; //Enable UEV events
}

void delayms(uint32_t msec){
	while (msec-- > 0){
		delayus(1000);
	}
}

void delayus(uint16_t usec){
	//TIM2->CR1 |= TIM_CR1_CEN;  //Enable TIM2
	if (usec <= 2){
		return;
	}else if (usec < 10){
		uint16_t counter = usec * 2 - 1;
		while(counter-- > 0)
			asm("nop");
		return;
	}
	init_TIM2_Change_Period(usec);
	while(TIM2_flag == 0){
		//__WFI();  //can cause debugger to think it has disconnected, explore http://nuttx.org/doku.php?id=wiki:howtos:jtag-debugging
	};
	//TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); //DISABLE TIM2
}

void startDelayus(uint16_t usec){
	init_TIM2_Change_Period(usec);
}

void waitSpecificCount(uint16_t usec){
	while(TIM_GetCounter(TIM2) < usec){};
}
void waitStartedDelay(void){
	while(TIM2_flag == 0){
		//__WFI();  //can cause debugger to think it has disconnected, explore http://nuttx.org/doku.php?id=wiki:howtos:jtag-debugging
	};
}


