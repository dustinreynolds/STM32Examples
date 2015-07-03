/*
 * packet_eeprom.c
 *
 *  Created on: Jun 28, 2015
 *      Author: dustin
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

#include "stdbool.h"
#include "stm32l1xx.h"
#include "packet_eeprom.h"
#include "crc_16bit_algorithm_dnp.h"
#include "uart.h"

void _packet_find_crc_with_stx(USART_TypeDef * USARTx){
	/*//CA D0 05 04 00 00 00 A5 C0 0A 3A
	//CA D0 05 04 00 00 00 EB C0 00 93
	//CA D0 05 04 00 00 01 31 C6 C0 0A
	//CA D0 05 04 00 00 01 64 96 C0 00
	//CA D0 05 04 00 00 01 B5 C0 0A 1B
	//CA D0 05 04 00 00 01 B8 67 C0 0A 	//CA D0 05 04 00 00 01 ED 37 C0 00 	//CA D0 05 04 00 00 01 FB C0 00 B2 
	//CA D0 05 04 00 00 02 85 C0 0A 78 
	//CA D0 05 04 00 00 02 CB C0 00 D1 	//CA D0 05 04 00 00 03 04 52 C0 0A 	//CA D0 05 04 00 00 03 51 02 C0 00 	//CA D0 05 04 00 00 03 8D F3 C0 0A 	//CA D0 05 04 00 00 03 95 C0 0A 59 	//CA D0 05 04 00 00 03 D8 A3 C0 00 	//CA D0 05 04 00 00 03 DB C0 00 F0 	//CA D0 05 04 00 00 04 1C FC C0 00 	//CA D0 05 04 00 00 04 49 AC C0 0A 	//CA D0 05 04 00 00 04 95 5D C0 00 	//CA D0 05 04 00 00 04 AB C0 00 17 	//CA D0 05 04 00 00 04 C0 0A 43 C0 0A 	//CA D0 05 04 00 00 04 E5 C0 0A BE 	//CA D0 05 04 00 00 05 BB C0 00 36 	//CA D0 05 04 00 00 05 F5 C0 0A 9F 	//CA D0 05 04 00 00 06 29 68 C0 00 	//CA D0 05 04 00 00 06 7C 38 C0 0A 	//CA D0 05 04 00 00 06 8B C0 00 55 	//CA D0 05 04 00 00 06 A0 C9 C0 00 	//CA D0 05 04 00 00 06 C5 C0 0A FC 	//CA D0 05 04 00 00 06 F5 99 C0 0A 	//CA D0 05 04 00 00 07 9B C0 00 74 	//CA D0 05 04 00 00 07 D5 C0 0A DD 	//CA D0 05 04 00 00 08 24 C0 0A 32 	//CA D0 05 04 00 00 08 6A C0 00 9B 	//CA D0 05 04 00 00 09 28 77 C0 00 	//CA D0 05 04 00 00 09 34 C0 0A 13 	//CA D0 05 04 00 00 09 7A C0 00 BA 	//CA D0 05 04 00 00 09 7D 27 C0 0A 	//CA D0 05 04 00 00 09 A1 D6 C0 00 	//CA D0 05 04 00 00 09 F4 86 C0 0A 	//CA D0 05 04 00 00 0A 04 C0 0A 70 	//CA D0 05 04 00 00 0A 4A C0 00 D9 	//CA D0 05 04 00 00 0B 14 C0 0A 51 	//CA D0 05 04 00 00 0B 1D E3 C0 00 	//CA D0 05 04 00 00 0B 48 B3 C0 0A 	//CA D0 05 04 00 00 0B 5A C0 00 F8 	//CA D0 05 04 00 00 0B 94 42 C0 00 	//CA D0 05 04 00 00 0B C1 12 C0 0A 	//CA D0 05 04 00 00 0C 05 4D C0 0A 	//CA D0 05 04 00 00 0C 2A C0 00 1F 	//CA D0 05 04 00 00 0C 50 1D C0 00 	//CA D0 05 04 00 00 0C 64 C0 0A B6 	//CA D0 05 04 00 00 0C 8C EC C0 0A 	//CA D0 05 04 00 00 0C D9 BC C0 00 	//CA D0 05 04 00 00 0D 3A C0 00 3E 	//CA D0 05 04 00 00 0D 74 C0 0A 97 	//CA D0 05 04 00 00 0E 0A C0 00 5D 	//CA D0 05 04 00 00 0E 30 D9 C0 0A 	//CA D0 05 04 00 00 0E 44 C0 0A F4 	//CA D0 05 04 00 00 0E 65 89 C0 00 	//CA D0 05 04 00 00 0E B9 78 C0 0A 	//CA D0 05 04 00 00 0E EC 28 C0 00 	//CA D0 05 04 00 00 0F 1A C0 00 7C 	//CA D0 05 04 00 00 0F 54 C0 0A D5 	//CA D0 05 04 00 00 10 32 E7 C0 0A 	//CA D0 05 04 00 00 10 67 B7 C0 00 	//CA D0 05 04 00 00 10 B7 C0 0A 0B 	//CA D0 05 04 00 00 10 BB 46 C0 0A 	//CA D0 05 04 00 00 10 EE 16 C0 00 	//CA D0 05 04 00 00 10 F9 C0 00 A2 	//CA D0 05 04 00 00 11 A7 C0 0A 2A 	//CA D0 05 04 00 00 11 E9 C0 00 83 	//CA D0 05 04 00 00 12 07 73 C0 0A 	//CA D0 05 04 00 00 12 52 23 C0 00 	//CA D0 05 04 00 00 12 8E D2 C0 0A 	//CA D0 05 04 00 00 12 97 C0 0A 49 	//CA D0 05 04 00 00 12 C0 00 C0 0A 9A 	//CA D0 05 04 00 00 12 D9 C0 00 E0 	//CA D0 05 04 00 00 12 DB 82 C0 00 	//CA D0 05 04 00 00 13 87 C0 0A 68 	//CA D0 05 04 00 00 13 C9 C0 00 C1 	//CA D0 05 04 00 00 14 B9 C0 00 26 	//CA D0 05 04 00 00 14 F7 C0 0A 8F 	//CA D0 05 04 00 00 15 1F DD C0 00 	//CA D0 05 04 00 00 15 4A 8D C0 0A 	//CA D0 05 04 00 00 15 96 7C C0 00 	//CA D0 05 04 00 00 15 A9 C0 00 07 	//CA D0 05 04 00 00 15 C3 2C C0 0A 	//CA D0 05 04 00 00 15 E7 C0 0A AE 	//CA D0 05 04 00 00 16 99 C0 00 64 	//CA D0 05 04 00 00 16 D7 C0 0A CD 	//CA D0 05 04 00 00 17 2A 49 C0 00 	//CA D0 05 04 00 00 17 7F 19 C0 0A 	//CA D0 05 04 00 00 17 89 C0 00 45 	//CA D0 05 04 00 00 17 A3 E8 C0 00 	//CA D0 05 04 00 00 17 C7 C0 0A EC 	//CA D0 05 04 00 00 17 F6 B8 C0 0A 	//CA D0 05 04 00 00 18 2B 56 C0 00 	//CA D0 05 04 00 00 18 36 C0 0A 03 	//CA D0 05 04 00 00 18 78 C0 00 AA 	//CA D0 05 04 00 00 18 7E 06 C0 0A 	//CA D0 05 04 00 00 18 A2 F7 C0 00 	//CA D0 05 04 00 00 18 F7 A7 C0 0A 	//CA D0 05 04 00 00 19 26 C0 0A 22 	//CA D0 05 04 00 00 19 68 C0 00 8B 	//CA D0 05 04 00 00 1A 16 C0 0A 41 	//CA D0 05 04 00 00 1A 1E C2 C0 00 	//CA D0 05 04 00 00 1A 4B 92 C0 0A 	//CA D0 05 04 00 00 1A 58 C0 00 E8 	//CA D0 05 04 00 00 1A 97 63 C0 00 	//CA D0 05 04 00 00 1A C2 33 C0 0A 	//CA D0 05 04 00 00 1B 06 C0 0A 60 	//CA D0 05 04 00 00 1B 48 C0 00 C9 	//CA D0 05 04 00 00 1C 38 C0 00 2E 	//CA D0 05 04 00 00 1C 76 C0 0A 87 	//CA D0 05 04 00 00 1D 06 6C C0 0A 	//CA D0 05 04 00 00 1D 28 C0 00 0F 	//CA D0 05 04 00 00 1D 53 3C C0 00 	//CA D0 05 04 00 00 1D 66 C0 0A A6 	//CA D0 05 04 00 00 1D 8F CD C0 0A 	//CA D0 05 04 00 00 1D DA 9D C0 00 	//CA D0 05 04 00 00 1E 18 C0 00 6C 	//CA D0 05 04 00 00 1E 56 C0 0A C5 	//CA D0 05 04 00 00 1F 08 C0 00 4D 	//CA D0 05 04 00 00 1F 33 F8 C0 0A 	//CA D0 05 04 00 00 1F 46 C0 0A E4 	//CA D0 05 04 00 00 1F 66 A8 C0 00 	//CA D0 05 04 00 00 1F BA 59 C0 0A 	//CA D0 05 04 00 00 1F EF 09 C0 00 	//CA D0 05 04 00 00 20 81 C0 0A 58 	//CA D0 05 04 00 00 20 CF C0 00 F1 	//CA D0 05 04 00 00 21 02 10 C0 0A 	//CA D0 05 04 00 00 21 57 40 C0 00 	//CA D0 05 04 00 00 21 8B B1 C0 0A 	//CA D0 05 04 00 00 21 91 C0 0A 79 	//CA D0 05 04 00 00 21 DE E1 C0 00 	//CA D0 05 04 00 00 21 DF C0 00 D0 	//CA D0 05 04 00 00 22 A1 C0 0A 1A 	//CA D0 05 04 00 00 22 EF C0 00 B3 	//CA D0 05 04 00 00 23 37 84 C0 0A 	//CA D0 05 04 00 00 23 62 D4 C0 00 	//CA D0 05 04 00 00 23 B1 C0 0A 3B 	//CA D0 05 04 00 00 23 BE 25 C0 0A 	//CA D0 05 04 00 00 23 EB 75 C0 00 	//CA D0 05 04 00 00 23 FF C0 00 92 	//CA D0 05 04 00 00 24 2F 2A C0 00 	//CA D0 05 04 00 00 24 7A 7A C0 0A 	//CA D0 05 04 00 00 24 8F C0 00 75 	//CA D0 05 04 00 00 24 A6 8B C0 00 	//CA D0 05 04 00 00 24 C1 C0 0A DC 	//CA D0 05 04 00 00 24 F3 DB C0 0A 	//CA D0 05 04 00 00 25 9F C0 00 54 	//CA D0 05 04 00 00 25 D1 C0 0A FD 	//CA D0 05 04 00 00 26 1A BE C0 00 	//CA D0 05 04 00 00 26 4F EE C0 0A 	//CA D0 05 04 00 00 26 93 1F C0 00 	//CA D0 05 04 00 00 26 AF C0 00 37 	//CA D0 05 04 00 00 26 C6 4F C0 0A 	//CA D0 05 04 00 00 26 E1 C0 0A 9E */
	parser_t dummy;
	uint32_t i;
	uint16_t dumpos;
	uint8_t dataBuffer[100];

	for (i = 0; i < 10000; i++){
		dummy.packet.id = 0xD0;
		dummy.packet.sub_id = 0x05;
		dummy.packet.length = 0x04;
		dummy.packet.payload[3] = i & 0xFF;
		dummy.packet.payload[2] = (i >> 8) & 0xFF;
		dummy.packet.payload[1] = (i>>16) & 0xFF;
		dummy.packet.payload[0] = (i>>24) & 0xFF;

		packet_eeprom_prepare_packet(&dummy, dataBuffer,&dumpos);
		if (((dummy.crc & 0xFF) == 0xCA) | ((dummy.crc & 0xFF) == 0xC0)){
			uart_OutBuffer(USARTx, dataBuffer, dumpos);
			//break;
		}else if (((dummy.crc & 0xFF00) == 0xCA00) | ((dummy.crc & 0xFF00) == 0xC000)){
			uart_OutBuffer(USARTx, dataBuffer, dumpos);
			//break;
		}
	}
}

void packet_eeprom_byte_stuff(uint8_t byte, uint8_t * buffer, uint16_t * index){
	if (byte == STX || byte == ESC){
		buffer[*index] = ESC;
		*index = *index + 1;
		buffer[*index] = byte ^ ESC;
		*index = *index + 1;
	}else{
		buffer[*index] = byte;
		*index = *index + 1;
	}
}

packet_eeprom_t packet_eeprom_read_packet(USART_TypeDef * USARTx, uint32_t *address, parser_t * details){
	//uint32_t data;
	uint16_t i, j = 0;
	bool keep_processing = true;
	packet_eeprom_t packet_result;
	//uint8_t dataBuffer[50];
	union {
		uint32_t u32;
		uint8_t u8[4];
	} data;

	data.u32 = *(__IO uint32_t*)*address;
	if (data.u8[0] != STX){
		return 1;
	}

	//feed each byte into packet parser. finished when encounter STX or when
	//packet is good.

	while (keep_processing){
		data.u32 = *(__IO uint32_t*)*address;
		for (i = 0; i < 4; i++){
			packet_result = packet_eeprom_parser(data.u8[i], details);

			if (packet_result == PKT_SUCCESS){
				uint8_t dataBuffer[100];
				uint16_t pos = 0;

				packet_eeprom_prepare_packet(details,dataBuffer, &pos);
				uart_OutBuffer(USARTx, dataBuffer, pos);
				return PKT_SUCCESS;
			}else if (packet_result == PKT_FAILURE){
				return PKT_FAILURE;
			}
		}
		*address = *address + 4;

		j += 4;
		if (j > MAX_PAYLOAD_SIZE){
			return PKT_FAILURE;
		}
	}
}

void packet_eeprom_write(uint8_t * buffer, uint16_t size, uint32_t *address){
	uint32_t data;
	uint16_t i, j;
	__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

	//enable flash writes here
	DATA_EEPROM_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
				| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

	//handle all 4 byte multiples
	for (i = 0, j = 0; i < (size/4); i++){
		data = (buffer[j+3] << 24) | (buffer[j+2] << 16) | (buffer[j+1] << 8) | buffer[j];

		do{
			FLASHStatus = DATA_EEPROM_ProgramWord(*address, data);

			if(FLASHStatus == FLASH_COMPLETE){
				*address = *address + 4;
			}else{
				FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
						| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
			}
		}while (FLASHStatus != FLASH_COMPLETE);
		j += 4;
	}
	//handle remainder
	if (j < size){
		data = buffer[j++];
		if (j<size){
			data = data | (buffer[j++] << 8);
			if(j<size){
				data = data | (buffer[j++] << 16);
				if(j<size){
					data = data | (buffer[j++] << 24);
				}
			}
		}
		do{
			FLASHStatus = DATA_EEPROM_ProgramWord(*address, data);

			if(FLASHStatus == FLASH_COMPLETE){
				*address = *address + 4;
			}else{
				FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
						| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
			}
		}while (FLASHStatus != FLASH_COMPLETE);
	}

	DATA_EEPROM_Lock();
}

void packet_eeprom_prepare_packet(parser_t * details, uint8_t * sendBuffer, uint16_t * pos){
	uint8_t i = 0;
	*pos = 0;

	//  Need to Stuff bytes
	sendBuffer[*pos] = STX;
	*pos = *pos +1;

	packet_eeprom_byte_stuff(details->packet.id, sendBuffer, pos);
	packet_eeprom_byte_stuff(details->packet.sub_id, sendBuffer, pos);
	packet_eeprom_byte_stuff(details->packet.length, sendBuffer, pos);

	for (i = 0; i < details->packet.length; i++){
		packet_eeprom_byte_stuff(details->packet.payload[i], sendBuffer, pos);
	}
	// Calculate CRC on stuffed bytes
	details->crc = crc_16bit_algorithm_dnp_calculate_full(sendBuffer, *pos, 0xFFFF, 0xFFFF);

	// Stuff CRC if needed
	packet_eeprom_byte_stuff((details->crc & 0xFF),sendBuffer, pos);
	packet_eeprom_byte_stuff((details->crc >> 8), sendBuffer, pos);
}

void packet_parser_init(parser_t * details){
	details->state = PKT_HEADER;
	details->byte_stuff = false;
}

packet_eeprom_t packet_eeprom_parser(uint8_t byte, parser_t * details){

	if (byte == STX){
		details->crc = crc_16bit_algorithm_dnp_update(0xFFFF, byte);
		details->packet.stx = STX;
		details->state = PKT_ID;
		return details->state;
	}

	//Add byte to current crc
	if (details->state > PKT_HEADER && details->state < PKT_CRC1){
		details->crc = crc_16bit_algorithm_dnp_update(details->crc, byte);
	}

	//Do byte stuffing
	if(details->byte_stuff){
		if (byte == ESC_NULL || byte == ESC_A){
			byte = ESC | byte;
			details->byte_stuff = false;
		}else{
			//Invalid case!
			details->state = PKT_HEADER;
			details->byte_stuff = false;
			return details->state;
		}
	}else if (byte == ESC){
		details->byte_stuff = true;
		return details->state;
	}

	//Process state
	if (details->state == PKT_ID){
		details->packet.id = byte;
		details->state = PKT_SUB_ID;
	}else if (details->state == PKT_SUB_ID){
		details->packet.sub_id = byte;
		details->state = PKT_LENGTH;
	}else if (details->state == PKT_LENGTH){
		details->received_length = 0;
		details->packet.length = byte;
		if (details->packet.length == 0x00){
			details->state = PKT_CRC1;
		}else{
			details->state = PKT_PAYLOAD;
		}
	}else if (details->state == PKT_PAYLOAD){
		details->packet.payload[details->received_length++] = byte;
		if (details->received_length == details->packet.length){
			details->state = PKT_CRC1;
		}
	}else if (details->state == PKT_CRC1){
		details->packet.crc[0] = byte;
		details->state = PKT_CRC2;
	}else if (details->state == PKT_CRC2){
		details->packet.crc[1] = byte;
		details->crc = details->crc ^ 0xFFFF;

		if (details->crc == ((details->packet.crc[1] << 8) | details->packet.crc[0])){
			details->state = PKT_SUCCESS;
		}else{
			details->state = PKT_FAILURE;
		}
	}else{
		details->state = PKT_HEADER;
	}
	return details->state;
}
