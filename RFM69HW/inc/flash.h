/*
 * flash.h
 *
 *  Created on: May 10, 2015
 *      Author: Dustin
 */

#ifndef FLASH_H_
#define FLASH_H_

uint8_t flash_present(void);
uint16_t flash_view_status(void);
void flash_enable_write(uint8_t enable);
void flash_read(uint8_t * data, uint32_t address, uint32_t size);
void flash_write_page(uint8_t * data, uint16_t address);
uint8_t flash_erase_sector(uint32_t address);
uint8_t flash_erase_all(void);
uint8_t flash_stress_test(void);

#endif /* FLASH_H_ */
