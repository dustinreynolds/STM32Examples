/*
 * spi.c
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

#include "stm32l1xx.h"
#include "spi.h"

static SPI_Device_t spi_devices[3];

uint8_t SPI2ReceivedValue[SPI2_BUFFER_SIZE];
uint8_t SPI2TransmitValue[SPI2_BUFFER_SIZE];

void spi_SPI2_Configuration(void){
	SPI_InitTypeDef SPI_InitStruct;

	/* Set All CS lines high */
	GPIOC->BSRRL |= GPIO_Pin_8;
	GPIOB->BSRRL |= GPIO_Pin_12;
	GPIOA->BSRRL |= GPIO_Pin_15;
	GPIOB->BSRRL |= GPIO_Pin_7;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //32/2 = 16MHz
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStruct);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel4); //SPI2 RX Channel

	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SPI2ReceivedValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = SPI2_BUFFER_SIZE;

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);


	DMA_DeInit(DMA1_Channel5); //SPI2 TX Channel
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SPI2TransmitValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = SPI2_BUFFER_SIZE;

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	spi_devices[SPI_FLASH].cs_port = GPIOC;
	spi_devices[SPI_FLASH].cs_pin = GPIO_Pin_8;
	spi_devices[SPI_FLASH].SPIx = SPI2;

	spi_devices[SPI_RFM69_1].cs_port = GPIOB;
	spi_devices[SPI_RFM69_1].cs_pin = GPIO_Pin_12;
	spi_devices[SPI_RFM69_1].SPIx = SPI2;
}

void spi_SPI3_Configuration(void){
	SPI_InitTypeDef SPI_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //32/2 = 16MHz
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI3, &SPI_InitStruct);

	SPI_Cmd(SPI3, ENABLE);

	spi_devices[SPI_RFM69_2].cs_port = GPIOA;
	spi_devices[SPI_RFM69_2].cs_pin = GPIO_Pin_15;
	spi_devices[SPI_RFM69_2].SPIx = SPI3;
}

uint8_t __inline__ spi_send(SPI_TypeDef* SPIx, uint8_t data){
	SPIx->DR = data;
	while(!(SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(SPIx->SR & SPI_I2S_FLAG_BSY);

	return SPIx->DR;
}

uint8_t spi_write_register(SPI_TypeDef* SPIx, uint8_t reg_cmd, uint8_t reg_op){
	uint8_t dummy;
	SPIx->DR = reg_cmd;
	while(!(SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(SPIx->SR & SPI_I2S_FLAG_BSY);

	dummy = SPIx->DR;
	SPIx->DR = reg_op;
	while(!(SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(SPIx->SR & SPI_I2S_FLAG_BSY);
	return SPIx->DR;
}

uint8_t spi_read_register(SPI_TypeDef* SPIx, uint8_t reg_cmd){
	uint8_t dummy;
	SPIx->DR = reg_cmd;
	while(!(SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(SPIx->SR & SPI_I2S_FLAG_BSY);
	dummy = SPIx->DR;

	SPIx->DR = 0x00;
	while(!(SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(SPIx->SR & SPI_I2S_FLAG_BSY);
	return SPIx->DR;
}
void spi_cs_activate(SPI_TypeDef* SPIx){
	if (SPIx == SPI2){
		GPIOB->BSRRH |= GPIO_Pin_12;
	} else if (SPIx == SPI3){
		GPIOA->BSRRH |= GPIO_Pin_15;
	}
}

void spi_cs_deactivate(SPI_TypeDef* SPIx){
	if (SPIx == SPI2){
		GPIOB->BSRRL |= GPIO_Pin_12;
	} else if (SPIx == SPI3){
		GPIOA->BSRRL |= GPIO_Pin_15;
	}
}

uint8_t spi_write_register_cs(uint8_t num, uint8_t reg_cmd, uint8_t reg_op){
	uint8_t dummy;
	spi_devices[num].cs_port->BSRRH |= spi_devices[num].cs_pin;
	spi_devices[num].SPIx->DR = reg_cmd;
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_BSY);

	dummy = spi_devices[num].SPIx->DR;
	spi_devices[num].SPIx->DR = reg_op;
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_BSY);
	dummy = spi_devices[num].SPIx->DR;
	spi_devices[num].cs_port->BSRRL |= spi_devices[num].cs_pin;
	return dummy;
}

uint8_t spi_read_register_cs(uint8_t num, uint8_t reg_cmd){
	uint8_t dummy = 0x00;
	spi_devices[num].cs_port->BSRRH |= spi_devices[num].cs_pin;
	spi_devices[num].SPIx->DR = reg_cmd;
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_BSY);
	dummy = spi_devices[num].SPIx->DR;

	spi_devices[num].SPIx->DR = 0x00;
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_TXE));
	while(!(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_RXNE));
	while(spi_devices[num].SPIx->SR & SPI_I2S_FLAG_BSY);
	dummy =  spi_devices[num].SPIx->DR;
	spi_devices[num].cs_port->BSRRL |= spi_devices[num].cs_pin;
	return dummy;
}

uint8_t __inline__ spi_send_buffer(uint32_t size){
	GPIOC->BSRRH |= GPIO_Pin_8;

	DMA_SetCurrDataCounter(DMA1_Channel5,size);
	DMA_SetCurrDataCounter(DMA1_Channel4,size);
	DMA_ClearFlag(DMA1_FLAG_GL5);
	DMA_Cmd(DMA1_Channel4, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

	while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET);

	GPIOC->BSRRL |= GPIO_Pin_8;

	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);

	return 1;
}

