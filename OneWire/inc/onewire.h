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

enum {
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

typedef enum {
	onewire_OW3 = 0x00, //PC6
	onewire_OW4 = 0x01, //PC7
} onewire_port_t;

void onewire_Init(onewire_port_t OWx);

uint8_t onewire_sendResetBasic(onewire_port_t OWx);
void onewire_read_latest_ROM(uint8_t * rom);
void onewire_read_temp(onewire_port_t OWx, uint8_t rom[8]);
void onewire_trigger_temp(onewire_port_t OWx);
void onewire_read_stored_temp(onewire_port_t OWx, uint8_t rom[8]);

// method declarations for Maxim 1-wire
int  OWFirst(onewire_port_t OWx);
int  OWNext(onewire_port_t OWx);
int  OWVerify(onewire_port_t OWx);
void OWTargetSetup(unsigned char family_code);
void OWFamilySkipSetup();

#endif /* ONEWIRE_H_ */
