/*
 * spi.c
 *
 *  Created on: May 9, 2015
 *      Author: Dustin
 */

#include "stm32l1xx.h"
#include "spi.h"


uint8_t SPI2ReceivedValue[SPI2_BUFFER_SIZE];
uint8_t SPI2TransmitValue[SPI2_BUFFER_SIZE];

void spi_SPI2_Configuration(void){
	SPI_InitTypeDef SPI_InitStruct;

	/* Set All CS lines high */
	GPIOC->BSRRL |= GPIO_Pin_8;
	GPIOB->BSRRL |= GPIO_Pin_12;

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

}

uint8_t __inline__ spi_send(uint8_t data){
	SPI2->DR = data;
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE));
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE));
	while(SPI2->SR & SPI_I2S_FLAG_BSY);

	return SPI2->DR;
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

