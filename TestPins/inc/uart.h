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

void uart_switch_mode(uart_init_mode_t mode);
void uart_Configuration(uart_init_mode_t mode);
void uart_OutString(char *s);

#endif /* UART_H_ */
