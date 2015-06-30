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

void packet_eeprom_send_packet(USART_TypeDef * USARTx, uint8_t id, uint8_t * payload, uint8_t size){
	uint16_t index = 0;
	uint8_t sendBuffer[MAX_PAYLOAD_SIZE+20];
	uint8_t i = 0;
	uint16_t calculated_crc;

	//  Need to Stuff bytes
	sendBuffer[index++] = STX;

	packet_eeprom_byte_stuff(id, sendBuffer,&index);
	packet_eeprom_byte_stuff(size, sendBuffer, &index);

	for (i = 0; i < size; i++){
		packet_eeprom_byte_stuff(payload[i], sendBuffer, &index);

		if (index > sizeof(sendBuffer)){
			//Something is wrong here, return without sending
			return;
		}
	}
	// Calculate CRC on stuffed bytes
	calculated_crc = crc_16bit_algorithm_dnp_calculate_full(sendBuffer, index, 0xFFFF, 0xFFFF);

	// Stuff CRC if needed
	packet_eeprom_byte_stuff((calculated_crc & 0xFF),sendBuffer, &index);
	packet_eeprom_byte_stuff((calculated_crc >> 8), sendBuffer, &index);

	uart_OutBuffer(USARTx, sendBuffer, index);

	//Send
}

void packet_eeprom_parser(USART_TypeDef * USARTx, uint8_t byte){
	static unsigned short crc = 0;
	static bool byte_stuff = false;
	static packet_t packet;
	static uint8_t received_length;

	static packet_eeprom_t state = PKT_HEADER;

	if (byte == STX){
		crc = crc_16bit_algorithm_dnp_update(0xFFFF, byte);
		state = PKT_ID;
		return;
	}

	//Add byte to current crc
	if (state > PKT_HEADER && state < PKT_CRC1){
		crc = crc_16bit_algorithm_dnp_update(crc, byte);
	}

	//Do byte stuffing
	if(byte_stuff){
		if (byte == ESC_NULL || byte == ESC_FIVE){
			byte = ESC | byte;
			byte_stuff = false;
		}else{
			//Invalid case!
			state = PKT_HEADER;
			return;
		}
	}else if (byte == ESC){
		byte_stuff = true;
		return;
	}

	//Process state
	if (state == PKT_ID){
		packet.id = byte;
		state = PKT_LENGTH;
	}else if (state == PKT_LENGTH){
		received_length = 0;
		packet.length = byte;
		if (packet.length == 0x00){
			state = PKT_CRC1;
		}else{
			state = PKT_PAYLOAD;
		}
	}else if (state == PKT_PAYLOAD){
		packet.payload[received_length++] = byte;
		if (received_length == packet.length){
			state = PKT_CRC1;
		}
	}else if (state == PKT_CRC1){
		packet.crc[0] = byte;
		state = PKT_CRC2;
	}else if (state == PKT_CRC2){
		uint16_t received_crc;
		packet.crc[1] = byte;
		received_crc = (packet.crc[1] << 8) | packet.crc[0];
		crc = crc ^ 0xFFFF;

		if (crc == received_crc){
			//crc matches.
			uart_OutString(USARTx,"CRC Matches!\r\n");
			packet_eeprom_send_packet(USARTx, packet.id, packet.payload,packet.length);
		}else{
			char buffer[40];
			sprintf(buffer,"CRC Fail, Calc %04x, Recv %04x\r\n",crc, received_crc);
			uart_OutString(USARTx, buffer);
		}
		state = PKT_HEADER;
	}
}
