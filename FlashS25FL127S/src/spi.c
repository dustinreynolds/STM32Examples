/*
 * spi.c
 *
 *  Created on: May 9, 2015
 *      Author: Dustin
 */

#include "stm32l1xx.h"

void spi_SPI2_Configuration(void){
	SPI_InitTypeDef SPI_InitStruct;

	/* Set All CS lines high */
	GPIOC->BSRRL |= GPIO_Pin_8;
	GPIOB->BSRRL |= GPIO_Pin_12;

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

	SPI_Cmd(SPI2, ENABLE);
}

uint8_t __inline__ spi_send(uint8_t data){
	SPI2->DR = data;
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE));
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE));
	while(SPI2->SR & SPI_I2S_FLAG_BSY);

	return SPI2->DR;
}


