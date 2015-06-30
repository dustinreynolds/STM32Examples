/*
 * packet_eeprom.h
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

#ifndef PACKET_EEPROM_H_
#define PACKET_EEPROM_H_

#define STX			0xCA
#define ESC			0xC0
#define ESC_NULL	0x00
#define ESC_FIVE	0x0A

#define MAX_PAYLOAD_SIZE    80

typedef enum {
	PKT_HEADER = 0x00,
	PKT_ID,
	PKT_LENGTH,
	PKT_PAYLOAD,
	PKT_CRC1,
	PKT_CRC2,
} packet_eeprom_t;

typedef struct {
	uint8_t id;
	uint8_t length;
	uint8_t payload[MAX_PAYLOAD_SIZE];
	uint8_t crc[2];
} packet_t;

typedef enum {
	PKT_EEP_ID_VERSION = 0x00,
	PKT_EEP_ID_HW_TEMP = 0x01,
	PKT_EEP_ID_HW_GPS  = 0x02,
	PKT_EEP_ID_HW_MOISTURE = 0x03,
} pkt_eep_id_t;

void packet_eeprom_parser(USART_TypeDef * USARTx, uint8_t byte);

#endif /* PACKET_EEPROM_H_ */
