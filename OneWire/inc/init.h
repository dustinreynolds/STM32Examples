/*
 * init.h
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */

#ifndef INIT_H_
#define INIT_H_

void init_RCC_Configuration(void);
void init_GPIO_Configuration(void);
void init_HSI(void);
void init_TIM2_Configuration(void);
void init_TIM2_Change_Period(uint16_t period);

void delayms(uint32_t msec);
void delayus(uint16_t usec);
void startDelayus(uint16_t usec);
void waitSpecificCount(uint16_t usec);
void waitStartedDelay(void);

#endif /* INIT_H_ */
