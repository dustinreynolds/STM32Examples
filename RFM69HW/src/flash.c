/*
 * Flash.c
 *
 *  Created on: May 10, 2015
 *      Author: Dustin Reynolds
 *
 * Copyright (c) 2015, Dustin Reynolds
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of [project] nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "stm32l1xx.h"
#include "stdio.h"
#include "string.h"
#include "spi.h"
#include "uart.h"

#define DEBUG_FLASH

#define FLASH_SR1_STATUS_REGISTER_WRITE_DISABLE 0x80
#define FLASH_SR1_PROGRAMMING_ERROR_OCCURRED 0x40
#define FLASH_SR1_ERASE_ERROR_OCCURRED 0x20
#define FLASH_SR1_BLOCK_PROTECTION_BP2 0x10
#define FLASH_SR1_BLOCK_PROTECTION_BP1 0x08
#define FLASH_SR1_BLOCK_PROTECTION_BP0 0x04
#define FLASH_SR1_WRITE_ENABLE_LATCH 0x02
#define FLASH_SR1_WRITE_IN_PROGRESS 0x01

#define FLASH_SR2_BLOCK_ERASE_SIZE 0x80
#define FLASH_SR2_PAGE_BUFFER_WRAP 0x40
#define FLASH_SR2_ERASE_SUSPEND 0x02
#define FLASH_SR2_PROGRAM_SUSPEND 0x01

enum {
	FLASH_WRITE_REGISTER = 0x01,
	FLASH_PAGE_PROGRAM = 0x02,
	FLASH_READ_DATA = 0x03,
	FLASH_WRITE_DISABLE = 0x04,
	FLASH_READ_STATUS_REGISTER_1 = 0x05,
	FLASH_WRITE_ENABLE = 0x06,
	FLASH_READ_STATUS_REGISTER_2 = 0x07,
	FLASH_4K_SECTOR_ERASE = 0x20,
	FLASH_BULK_ERASE = 0x60,
	FLASH_READ_MANUFACTURE_SIGNATURE = 0x90,
};

extern uint8_t SPI2ReceivedValue[SPI2_BUFFER_SIZE];
extern uint8_t SPI2TransmitValue[SPI2_BUFFER_SIZE];

uint8_t flash_present(void){
	uint8_t success_flag = 0x00;

	memset(&SPI2TransmitValue, 0, 8);
	SPI2TransmitValue[0] = FLASH_READ_MANUFACTURE_SIGNATURE;

	spi_send_buffer(8);

	if (SPI2ReceivedValue[4] == 0x01 && SPI2ReceivedValue[6] == 0x01){
		success_flag |= 0x01;
	}
	if (SPI2ReceivedValue[5] == 0x17 && SPI2ReceivedValue[7] == 0x17){
		success_flag |= 0x02;
	}

	if (success_flag == 0x03)
		return 1;
	return 0;
}

uint16_t flash_view_status(void){
	uint8_t flash_reg[2];

	SPI2TransmitValue[0] = FLASH_READ_STATUS_REGISTER_1;
	SPI2TransmitValue[1] = 0x00;
	spi_send_buffer(2);
	flash_reg[0] = SPI2ReceivedValue[1];

	SPI2TransmitValue[0] = FLASH_READ_STATUS_REGISTER_2;
	SPI2TransmitValue[1] = 0x00;
	spi_send_buffer(2);
	flash_reg[1] = SPI2ReceivedValue[1];

#ifdef DEBUG_FLASH
	{
		char buffer[20];
		sprintf(buffer,"Flash_status: %d %d\r\n",flash_reg[0], flash_reg[1]);
		uart_OutString(buffer);
	}
#endif

	return (flash_reg[1] << 8 | flash_reg[0]); //reg2 reg1
}

void flash_enable_write(uint8_t enable){
	GPIOC->BSRRH |= GPIO_Pin_8;
	if (enable)
		spi_send(SPI2, FLASH_WRITE_ENABLE);
	else
		spi_send(SPI2, FLASH_WRITE_DISABLE);
	GPIOC->BSRRL |= GPIO_Pin_8;
}

void flash_read(uint8_t * data, uint32_t address, uint32_t size){

	SPI2TransmitValue[0] = FLASH_READ_DATA;
	SPI2TransmitValue[1] = (address >> 8 & 0xFF);
	SPI2TransmitValue[2] = (address & 0xFF);
	SPI2TransmitValue[3] = 0x00;
	memcpy(&SPI2TransmitValue[4], data, sizeof(uint8_t) * size);

	spi_send_buffer(size + 4);

	memcpy(data, &SPI2ReceivedValue[4], sizeof(uint8_t) * 256);

	memset(&SPI2ReceivedValue, 0, sizeof(SPI2ReceivedValue));
}

void flash_read_nonDMA(uint8_t * data, uint32_t address, uint32_t size){
	uint32_t i;
	GPIOC->BSRRH |= GPIO_Pin_8;
	spi_send(SPI2, FLASH_READ_DATA);
	spi_send(SPI2, (address >> 8 & 0xFF));
	spi_send(SPI2, (address & 0xFF));
	spi_send(SPI2, 0x00);
	for(i = 0; i < size; i++){
		*data++ = spi_send(SPI2, 0x00);
	}
	GPIOC->BSRRL |= GPIO_Pin_8;
}

void flash_write_page(uint8_t * data, uint16_t address){
	uint8_t flash_sr1;
	uint16_t timeout = 0;
	flash_enable_write(1);

	SPI2TransmitValue[0] = FLASH_PAGE_PROGRAM;
	SPI2TransmitValue[1] = (address >> 8 & 0xFF);
	SPI2TransmitValue[2] = (address & 0xFF);
	SPI2TransmitValue[3] = 0x00;
	memcpy(&SPI2TransmitValue[4], data, sizeof(uint8_t) * 256);

	spi_send_buffer(SPI2_BUFFER_SIZE);

	flash_enable_write(0);

	//Wait for it to finish
	while(timeout < 100)
	{
		SPI2TransmitValue[0] = FLASH_READ_STATUS_REGISTER_1;
		SPI2TransmitValue[1] = 0x00;
		spi_send_buffer(2);
		flash_sr1 = SPI2ReceivedValue[1];

		if ((flash_sr1 & FLASH_SR1_WRITE_IN_PROGRESS) == 0x00){
			break;
		}
		timeout++;
	}
}

uint8_t flash_erase_sector(uint32_t address){
	uint8_t flash_sr1;
	uint16_t timeout = 0;
	flash_enable_write(1);

	SPI2TransmitValue[0] = FLASH_4K_SECTOR_ERASE;
	SPI2TransmitValue[1] = (address >> 8 & 0xFF);
	SPI2TransmitValue[2] = (address & 0xFF);
	SPI2TransmitValue[3] = 0x00;
	spi_send_buffer(4);

	//Wait for it to finish
	while(1)
	{
		SPI2TransmitValue[0] = FLASH_READ_STATUS_REGISTER_1;
		SPI2TransmitValue[1] = 0x00;
		spi_send_buffer(2);
		flash_sr1 = SPI2ReceivedValue[1];
		if ((flash_sr1 & FLASH_SR1_WRITE_IN_PROGRESS) == 0x00){
			break;
		}
		timeout++;
	}
	flash_enable_write(0);

	{
		char buffer[30];
		sprintf(buffer,"ES: %d\r\n",timeout);
		uart_OutString(buffer);
	}
	return 1;
}

uint8_t flash_erase_all(void){
	uint8_t flash_sr1;
	uint32_t timeout = 0;
	uint8_t success_ctr = 0;
	flash_enable_write(1);
	GPIOC->BSRRH |= GPIO_Pin_8;
	spi_send(SPI2, FLASH_BULK_ERASE);
	GPIOC->BSRRL |= GPIO_Pin_8;

	//Wait for it to finish
	while(timeout < 5000000)
	{
		SPI2TransmitValue[0] = FLASH_READ_STATUS_REGISTER_1;
		SPI2TransmitValue[1] = 0x00;
		spi_send_buffer(2);
		flash_sr1 = SPI2ReceivedValue[1];
		if ((flash_sr1 & FLASH_SR1_WRITE_IN_PROGRESS) == 0x00){
			success_ctr++;
		}else{
			success_ctr = 0;
		}

		if (success_ctr > 10)
			break;
		timeout++;
	}
	flash_enable_write(0);

	if(timeout > 5000000)
		return 0;
	return 1;
}

uint8_t flash_stress_test(void){
	uint8_t timeout;
	uint8_t data[256];
	char buffer[100];
	uint32_t page = 0;
#ifdef DEBUG_FLASH
	uart_OutString("SPI Flash Erase All: ");
#endif
	timeout = flash_erase_all();
	if(timeout == 0){
#ifdef DEBUG_FLASH
		while(1){
			uart_OutString("Erase All Timed Out\r\n");
		}
#else
		return 0;//failed
#endif
	}else{
#ifdef DEBUG_FLASH
		uart_OutString("Erase All Successful\r\n");
#endif
	}

	flash_view_status();
#ifdef DEBUG_FLASH
	uart_OutString("Flash Stress Test In Progress\r\n");
#endif
	for(page = 0; page < 0xffff; page+=100)
	{
#ifdef DEBUG_FLASH
		if ((page % 1000) == 0){
			sprintf(buffer,"Page %d complete without error\r\n",page);
			uart_OutString(buffer);
		}
#endif
		{
			uint16_t i;
			flash_read(&data[0],page, 5);
			for (i = 0; i < 5; i++){
				if (data[i] != 0xFF)
				{
#ifdef DEBUG_FLASH
					sprintf(buffer,"Not Erased! Page %d, i=%02x != %02x\r\n",page, 0xff,data[i]);
					uart_OutString(buffer);
					while(1);
#else
					return 0;
#endif
				}
			}
		}

		{
			uint16_t i;
			for(i = 0; i < sizeof(data); i++){
				data[i] = i;
			}
			flash_write_page(&data[0], page);
			memset(data, 0, sizeof(data));
		}
		{
			uint16_t i;
			flash_read(&data[0],page, 256); //160us for transferring 260 bytes
			//flash_read_nonDMA(&data[0],page, 256); //900us for transferring 260 bytes
			for (i = 0; i < 5; i++){
				if (data[i] != i)
				{
#ifdef DEBUG_FLASH
					sprintf(buffer,"Not Match! Page %d, i=%02x != %02x\r\n",page, i,data[i]);
					uart_OutString(buffer);
					while(1);
#else
					return 0;
#endif
				}
			}
		}
	}
#ifdef DEBUG_FLASH
	sprintf(buffer,"Stress Test Completed, no errors\r\n");
	uart_OutString(buffer);
#endif

	return 1;
}
