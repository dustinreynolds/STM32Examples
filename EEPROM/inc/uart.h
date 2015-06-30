/*
 * uart.h
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */

#ifndef UART_H_
#define UART_H_

typedef enum {
	UART_INTERRUPT_RX,
	UART_POLLING
} uart_init_mode_t;

void uart_switch_mode(USART_TypeDef * USARTx,uart_init_mode_t mode);
void uart_Configuration(USART_TypeDef * USARTx, uart_init_mode_t mode);
void uart_OutString(USART_TypeDef * USARTx, char *s);
void uart_OutBuffer(USART_TypeDef * USARTx, uint8_t *s, uint16_t size);

#endif /* UART_H_ */
