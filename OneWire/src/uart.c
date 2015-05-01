/*
 * uart.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"

void USART2_IRQHandler(void)
{
	if( USART_GetITStatus(USART2, USART_IT_RXNE)){
		char t = USART_ReceiveData(USART2);

		//do something.
		USART_SendData(USART2, (t+10));
	}
}


void uart_NVIC_USART2_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void uart_NVIC_USART2_deinit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void uart_switch_mode(uart_init_mode_t mode){
	if (mode == UART_INTERRUPT_RX){
		uart_NVIC_USART2_init();
	}else{
		uart_NVIC_USART2_deinit();
	}
}

void uart_Configuration(uart_init_mode_t mode)
{
	USART_InitTypeDef USART_InitStructure;
	if (mode == UART_INTERRUPT_RX){
		uart_NVIC_USART2_init();
	}else{
		uart_NVIC_USART2_deinit();
	}

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);

	if (mode == UART_INTERRUPT_RX){
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}
}


void uart_OutString(char *s)
{
	while(*s)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, *s++); // Send Char
	}
}


