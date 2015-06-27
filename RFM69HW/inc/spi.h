/*
 * spi.h
 *
 *  Created on: May 9, 2015
 *      Author: Dustin
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

#ifndef SPI_H_
#define SPI_H_

#define SPI_MAX_NUM_DEVICES 3
#define SPI_FLASH				0
#define SPI_RFM69_1				1
#define SPI_RFM69_2				2

#define SPI2_BUFFER_SIZE 260

typedef struct{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef * cs_port;
	uint16_t cs_pin;
} SPI_Device_t;

void spi_SPI2_Configuration(void);
void spi_SPI3_Configuration(void);
uint8_t spi_send(SPI_TypeDef* SPIx,uint8_t data);
uint8_t __inline__ spi_send_buffer(uint32_t size);
uint8_t __inline__ spi_send_dummy_buffer(uint32_t size);
uint8_t spi_write_register_cs(uint8_t num, uint8_t reg_cmd, uint8_t reg_op);
uint8_t spi_read_register_cs(uint8_t num, uint8_t reg_cmd);

#endif /* SPI_H_ */
