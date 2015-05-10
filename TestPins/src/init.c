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

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
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


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRRL |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 | GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRL |=GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14|GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->BSRRL |=GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIOD->BSRRL |=GPIO_Pin_2|GPIO_Pin_8;
}

void set_all_high(void){
	GPIOA->BSRRL |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 | GPIO_Pin_15;
	GPIOB->BSRRL |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14|GPIO_Pin_15;
	GPIOC->BSRRL |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13;
	GPIOD->BSRRL |= GPIO_Pin_2 | GPIO_Pin_8;

}

void set_all_low(void){
	GPIOA->BSRRH |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 | GPIO_Pin_15;
	GPIOB->BSRRH |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14|GPIO_Pin_15;
	GPIOC->BSRRH |= GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13;
	GPIOD->BSRRH |= GPIO_Pin_2 | GPIO_Pin_8;
}
