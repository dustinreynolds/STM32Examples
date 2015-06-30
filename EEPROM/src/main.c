/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "uart.h"
#include "packet_eeprom.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080803FF
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus DataMemoryProgramStatus = PASSED;
uint32_t NbrOfPage = 0, j = 0, Address = 0;

int main(void)
{
	USART_TypeDef * USARTx = USART2;
	init_RCC_Configuration();

	init_GPIO_Configuration();

	uart_Configuration(USARTx, UART_POLLING);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	//unlock eeprom
	DATA_EEPROM_Unlock();

	/* Clear all pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

	/*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by
	      DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */
	Address = DATA_EEPROM_START_ADDR;

	NbrOfPage = ((DATA_EEPROM_END_ADDR - Address) + 1 ) >> 2;

	/* Erase the Data EEPROM Memory pages by Word (32-bit) */
	for(j = 0; j < NbrOfPage; j++)
	{
		FLASHStatus = DATA_EEPROM_EraseWord(Address + (4 * j));
	}

	/* Check the correctness of written data */
	while(Address < DATA_EEPROM_END_ADDR)
	{
		if(*(__IO uint32_t*)Address != 0x0)
		{
			DataMemoryProgramStatus = FAILED;
		}
		Address = Address + 4;
	}

	Address = DATA_EEPROM_START_ADDR;

	/* Program the Data EEPROM Memory pages by Word (32-bit) */
	while(Address <= DATA_EEPROM_END_ADDR )
	{
		FLASHStatus = DATA_EEPROM_FastProgramWord(Address, FAST_DATA_32);

		if(FLASHStatus == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
					| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
		}
	}

	Address = DATA_EEPROM_START_ADDR;

	/* Check the correctness of written data */
	while(Address < DATA_EEPROM_END_ADDR)
	{
		if(*(__IO uint32_t*)Address != FAST_DATA_32)
		{
			DataMemoryProgramStatus = FAILED;
		}
		Address = Address + 4;
	}

	/*  Data EEPROM Word program of DATA_32 at addresses defined by DATA_EEPROM_START_ADDR
	      and DATA_EEPROM_END_ADDR */
	Address = DATA_EEPROM_START_ADDR;

	while(Address <= DATA_EEPROM_END_ADDR )
	{
		FLASHStatus = DATA_EEPROM_ProgramWord(Address, DATA_32);

		if(FLASHStatus == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
					| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
		}
	}

	Address = DATA_EEPROM_START_ADDR;

	/* Check the correctness of written data */
	while(Address < DATA_EEPROM_END_ADDR)
	{
		if(*(__IO uint32_t*)Address != DATA_32)
		{
			DataMemoryProgramStatus = FAILED;
		}
		Address = Address + 4;
	}

	if (DataMemoryProgramStatus != FAILED)
	{
		/* OK */
		/* Turn on LD1 */
		//STM_EVAL_LEDOn(LED1);
		uart_OutString(USARTx,"Successfully programmed eeprom");
	}
	else
	{
		/* KO */
		/* Turn on LD2 */
		//STM_EVAL_LEDOn(LED2);
		uart_OutString(USARTx,"Failed programming eeprom");
	}

	while(1){

		uint16_t Data;

		/*Only get a character if it is ready*/
		if(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) != RESET){

			Data = USART_ReceiveData(USARTx); // Collect Char

			while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait for Empty

			//USART_SendData(USARTx, Data); // Echo Char
			packet_eeprom_parser(USARTx, Data);
		}
	}
}
