/*
 * OneWire.h
 *
 *  Created on: Apr 25, 2015
 *      Author: Dustin
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#define OW3_MAX_SENSORS 6

#define OW_FALSE 0
#define OW_TRUE  1

typedef enum {
	onewire_read_rom = 0x33,
	onewire_match_rom = 0x55,
	onewire_skip_rom = 0xCC,
	onewire_alarm_search = 0xEC,
	onewire_search_rom = 0xF0,
};

typedef enum {
	DS18B20_convert = 0x44,
	DS18B20_write_scratchpad = 0x4E,
	DS18B20_read_scratchpad = 0xBE,
	DS18B20_copy_scratchpad = 0x48,
	DS18B20_recall_e_squared = 0xB8,
	DS18B20_read_powersupply = 0xB4,
};

typedef enum {
	onewire_high = 0x00,  //Master releases the bus
	onewire_low = 0x01,   //Master pulls down bus
} onewire_level_t;

void delayus(uint16_t usec);
void delayms(uint32_t msec);
void onewire_OW3Init(void);
void onewire_TIM2_Configuration(void);
void onewire_read_latest_ROM(uint8_t * rom);
void onewire_read_temp(uint8_t rom[8]);

// method declarations for Maxim 1-wire
int  OWFirst();
int  OWNext();
int  OWVerify();
void OWTargetSetup(unsigned char family_code);
void OWFamilySkipSetup();

#endif /* ONEWIRE_H_ */
