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
		USART_SendData(USART2, t);
	}
}

void USART1_IRQHandler(void)
{
	if( USART_GetITStatus(USART1, USART_IT_RXNE)){
		char t = USART_ReceiveData(USART1);

		//do something.
		USART_SendData(USART1, (t+10));
	}
}

void UART5_IRQHandler(void)
{
	if( USART_GetITStatus(UART5, USART_IT_RXNE)){
		char t = USART_ReceiveData(UART5);

		//do something.
		USART_SendData(USART2, t);
	}
}

void uart_NVIC_init(USART_TypeDef * USARTx)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	if (USARTx == USART1){
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	}else if (USARTx == USART2){
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	}else if (USARTx == UART5){
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void uart_NVIC_deinit(USART_TypeDef * USARTx)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	if (USARTx == USART1){
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	}else if (USARTx == USART2){
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	}else if (USARTx == UART5){
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void uart_switch_mode(USART_TypeDef * USARTx,uart_init_mode_t mode){
	if (mode == UART_INTERRUPT_RX){
		uart_NVIC_init(USARTx);
	}else{
		uart_NVIC_deinit(USARTx);
	}
}

void uart_Configuration(USART_TypeDef * USARTx, uart_init_mode_t mode)
{
	USART_InitTypeDef USART_InitStructure;
	if (mode == UART_INTERRUPT_RX){
		uart_NVIC_init(USARTx);
	}else{
		uart_NVIC_deinit(USARTx);
	}

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USARTx, &USART_InitStructure);

	USART_Cmd(USARTx, ENABLE);

	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
}

void uart_OutString(USART_TypeDef * USARTx, char *s)
{
	while(*s)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USARTx, *s++); // Send Char
	}
}

void uart_OutBuffer(USART_TypeDef * USARTx, uint8_t *s, uint16_t size){
	while(size-- > 0)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USARTx, *s++); // Send Char
	}
}


