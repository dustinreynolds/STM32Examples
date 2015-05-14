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
void delayms(uint32_t msec);

#endif /* INIT_H_ */
