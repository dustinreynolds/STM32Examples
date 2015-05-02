/*
 * OneWire.h
 *
 *  Created on: Apr 25, 2015
 *      Author: Dustin
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

typedef enum {
	onewire_high = 0x00,  //Master releases the bus
	onewire_low = 0x01,   //Master pulls down bus
} onewire_level_t;

void delayus(uint16_t usec);
void delayms(uint32_t msec);
void onewire_OW3Init(void);
void onewire_TIM2_Configuration(void);
void onewire_OW3Write(onewire_level_t state);

#endif /* ONEWIRE_H_ */
