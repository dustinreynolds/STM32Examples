/*
 * sx1231h_test.c
 *
 *  Created on: Jun 27, 2015
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
#include <string.h>
#include <stdio.h>
#include "stm32l1xx.h"
#include "sx1231h.h"
#include "spi.h"
#include "uart.h"
#include "init.h"
#include "sx1231h_test.h"

uint8_t sx1231h_test(test_strings_t testNumber, uint8_t dev1, uint8_t dev2){
	uint8_t result = 0;
	static const pTable_t testTable[] = {
			{ TEST_STRINGS_TEST_PRESENCE, &sx1231h_test_presence},
			{ TEST_STRINGS_BASIC_RX_TX, &sx1231h_test_single_change},
			{ TEST_STRINGS_DIFF_SYNC, &sx1231h_test_failure},
			{ TEST_STRINGS_AES_ON, &sx1231h_test_single_change},
			{ TEST_STRINGS_AES_DIFF, &sx1231h_test_single_change},
			{ TEST_STRINGS_NODE_ADDRESS, &sx1231h_test_single_change},
			{ TEST_STRINGS_BROADCAST_ADDRESS, &sx1231h_test_single_change},
			{ TEST_STRINGS_NODE_WRONG_ADDRESS, &sx1231h_test_failure},
			{ TEST_STRINGS_BROADCAST_NOT_ENABLED, &sx1231h_test_failure},
			{ TEST_STRINGS_RSSI_THRESHOLD, &sx1231h_test_rssi},
			{ TEST_STRINGS_FREQ_HOP, &sx1231h_test_freq_hop},
			{ TEST_STRINGS_FREQ_HOP_FAILURE, &sx1231h_test_freq_hop},
			{ TEST_STRINGS_RX_TIMEOUT_HIGH, &sx1231h_test_single_change},
			{ TEST_STRINGS_RX_TIMEOUT_FAILURE, &sx1231h_test_failure},
	};

	if (testNumber >= TEST_STRINGS_VOID){
		return ERROR;
	}

	//Execute Function
	result = (*testTable[testNumber].function)(testTable[testNumber].testIdentifier, dev1, dev2);

	//return result;
	return result;
}

typedef enum {
	ERROR_STRING_DEVICE_1_NOT_DETECTED 		= 0,
	ERROR_STRING_DEVICE_2_NOT_DETECTED,
	ERROR_STRING_TIMEOUT_DEVICE_1,
	ERROR_STRING_TX_TIMEOUT,
	ERROR_STRING_RX_TIMEOUT,
	ERROR_STRING_TIMEOUT,
	ERROR_STRING_RX_SIZE_DIFFERENT,
	ERROR_STRING_CORRUPTED_BYTES,
	ERROR_STRING_SHOULD_HAVE_FAILED,
	ERROR_STRING_AES_STRINGS_TOO_SIMILIAR,
} error_strings_t;

void sx1231h_error_strings(error_strings_t num, bool printCRLF){
	const char *const crlf = "\r\n";
	const char *const error_strings[] = {
			" test failed Device 1 not detected",
			" test failed Device 2 not detected",
			" test failed due to timeout for dev1",
			" test failed due to TX timeout",
			" test failed due to RX timeout",
			" test failed due to timeout.",
			" test failed RX size different",
			" test failed corrupted bytes in payload",
			" test failed due to successfully received packet",
			" test failed AES encrypted strings too similar",
	};

	uart_OutString(error_strings[num]);
	if (printCRLF){
		uart_OutString(crlf);
	}
}

typedef enum {
	COMMON_STRINGS_TEST_SUCCESS = 0,


} common_strings_t;

void sx1231h_common_strings(common_strings_t num, bool printCRLF){
	const char *const crlf = "\r\n";
	const char *const common_strings[] = {
			" test succeeded"
	};
	uart_OutString(common_strings[num]);
	if (printCRLF){
		uart_OutString(crlf);
	}
}

void sx1231h_test_identifier(test_strings_t num, bool printCRLF){
	const char *const crlf = "\r\n";
	const char *const test_strings[] = {
			"Presence",
			"RX TX",
			"Different Sync",
			"AES Enabled",
			"AES Different",
			"Node Address",
			"Broadcast Address",
			"Wrong Node Address",
			"Broadcast Address Not Enabled",
			"Minimum RSSI Threshold",
			"Frequency Hopping Both",
			"Frequency Hopping TX",
			"Rx Timeout High",
			"Rx Timeout Failure",
			"Test Void!"
	};

	if (num >= TEST_STRINGS_VOID){
		return;
	}

	uart_OutString(test_strings[num]);
	if (printCRLF){
		uart_OutString(crlf);
	}
}


//Test different lengths, versus bitrate, versus power, to determine the minimum power needed at each bitrate, to send a given length message
uint8_t sx1231h_test_find_lowest_settings(uint8_t dev1, uint8_t dev2){
	uint8_t i_len, i_rate, i_pow, i_dev;
	uint8_t score = 0;
	uint8_t iterations;

	uint8_t lengths[15] = {250,200,150,100,64,60,50,40,30,20,10,5,3,2,1};
	uint32_t baudrate[15] = {490,1200,4800,9600,19200,38400,57600,76800,115200,150000,200000,250000,300000,460800,921000};
	uint16_t freq_dev[15] = {5,10,20,40,50,60,75,100,200,300};
	int8_t power[15] = {20,19,18,17,15,14,13,10,5,2,0,-5,-10,-15,-18};

	sx1231h_dump_select_regs();

	for(i_len = 4; i_len < 15; i_len+=1){ //test different length
		for(i_rate = 2; i_rate < 14; i_rate++){ //test different bitrate
			for (i_dev = 0; i_dev < 10; i_dev++){
				for(i_pow = 0; i_pow < 15; i_pow+=1){ //Test different power
					for(iterations = 0; iterations < 10; iterations++){
						{
							uint8_t sendBuff[255], receiveBuff[255];
							uint8_t sendSize, receiveSize;
							uint8_t result;
							uint8_t i;

							result = sx1231h_init(dev1);
							if (result == ERROR){
								uart_OutString("FLS test failed Device 1 not detected\r\n");
								sx1231h_dump_select_regs();
								return ERROR;
							}
							result = sx1231h_init(dev2);
							if (result == ERROR){
								uart_OutString("FLS test failed Device 2 not detected\r\n");
								sx1231h_dump_select_regs();
								return ERROR;
							}
							sendSize = lengths[i_len];
							for(i = 0; i < sendSize; i++){
								sendBuff[i] = i;
							}

							//change bitrate
							sx1231h_set_baudrate(dev1,baudrate[i_rate]);
							sx1231h_set_baudrate(dev2,baudrate[i_rate]);

							//change power on tx
							sx1231h_set_power(dev1, power[i_pow]);

							//change freq dev
							sx1231h_set_freq_dev(dev1, freq_dev[i_dev]*1000);
							sx1231h_set_freq_dev(dev2, freq_dev[i_dev]*1000);

							//delayms(10);

							sx1231h_receiveFrameStart(dev2);
							result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
							if (result == ERROR){
								uart_OutString("FLS test failed due to timeout for dev1\r\n");
								sx1231h_dump_select_regs();
								return ERROR;
							}

							result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
							if (result == ERROR){
								//uart_OutString("FLS test failed due to timeout.\r\n");
								//return ERROR;
								//char buffer[40];
								//sprintf(buffer,"FLS Failed at %03d %06ld %06ld %+02d\r\n",lengths[i_len],baudrate[i_rate], freq_dev[i_dev]*1000, power[i_pow]);
								//uart_OutString(buffer);
								//sx1231h_dump_select_regs();
								//break;
								continue;
							}

							result = sx1231h_sendFrameWait(dev1);
							if (result == ERROR){
								sx1231h_dump_select_regs();
								uart_OutString("FLS test failed due to TX timeout.\r\n");
								return ERROR;
							}

							if (receiveSize != sendSize){
								sx1231h_dump_select_regs();
								uart_OutString("FLS test failed RX size diff.\r\n");
								return ERROR;
							}

							for(i = 0; i < receiveSize; i++){
								if (sendBuff[i] != receiveBuff[i]){
									sx1231h_dump_select_regs();
									uart_OutString("FLS test failed corrupted bytes in payload\r\n");
									return ERROR;
								}
							}
							{
								//char buffer[40];
								score++;
								//sprintf(buffer,"FLS Passed %03d %06ld %06ld %+02d\r\n",lengths[i_len],baudrate[i_rate], freq_dev[i_dev]*1000, power[i_pow]);
								//uart_OutString(buffer);
							}
						}
					}
					char buffer[40];
					sprintf(buffer,"FLS score:%d,%03d,%06ld,%06d,%+02d\r\n",score, lengths[i_len],baudrate[i_rate], freq_dev[i_dev]*1000, power[i_pow]);
					uart_OutString(buffer);
					//if (score == 0){
					//	break;
					//}
					score = 0;
				}
			}
		}
	}
	uart_OutString("FLS High Speed test finished.\r\n");
	return OK;
}

uint8_t sx1231h_test_presence(test_strings_t identifier, uint8_t dev1, uint8_t dev2){
	if(sx1231h_present(dev1) == 0){
		return ERROR_STRING_DEVICE_1_NOT_DETECTED;
	}
	if(sx1231h_present(dev2) == 0){
		return ERROR_STRING_DEVICE_2_NOT_DETECTED;
	}
	return OK;
}

uint8_t sx1231h_test_single_change(test_strings_t identifier, uint8_t dev1, uint8_t dev2){
	uint8_t sendBuff[30], receiveBuff[30];
	uint8_t sendSize, receiveSize;
	uint8_t result;
	uint8_t i;
	int8_t rssi;
	uint8_t counter = 0;

	result = sx1231h_init(dev1);
	if (result == ERROR){
		return ERROR_STRING_DEVICE_1_NOT_DETECTED;
	}
	result = sx1231h_init(dev2);
	if (result == ERROR){
		return ERROR_STRING_DEVICE_2_NOT_DETECTED;
	}
	sendSize = 30;
	for(i = 0; i < sendSize; i++){
		sendBuff[i] = i;
	}

	if (identifier == TEST_STRINGS_DIFF_SYNC){
		spi_write_register_cs(dev1, REG_SYNCVALUE4 | 0x80, 0x95);  //Byte 4 typically is 0x96
	}

	if (identifier == TEST_STRINGS_AES_ON){
		uint8_t aes_key[16];
		//Send AES key
		i=0;
		aes_key[i++] = 0x12;
		aes_key[i++] = 0x45;
		aes_key[i++] = 0x87;
		aes_key[i++] = 0xab;

		aes_key[i++] = 0xd4;
		aes_key[i++] = 0xb7;
		aes_key[i++] = 0x00;
		aes_key[i++] = 0x32;

		aes_key[i++] = 0x92;
		aes_key[i++] = 0x19;
		aes_key[i++] = 0xbf;
		aes_key[i++] = 0x56;

		aes_key[i++] = 0x89;
		aes_key[i++] = 0xa9;
		aes_key[i++] = 0xcc;
		aes_key[i++] = 0xdd;
		sx1231h_set_encryption_key(dev1, aes_key);
		sx1231h_set_encryption_key(dev2, aes_key);

		//Enable AES
		sx1231h_set_encryption_state(dev1, true);
		sx1231h_set_encryption_state(dev2, true);
	}else if (identifier == TEST_STRINGS_AES_DIFF){
		uint8_t aes_key[16];
		i=0;

		aes_key[i++] = 0x12;
		aes_key[i++] = 0x45;
		aes_key[i++] = 0x87;
		aes_key[i++] = 0xab;

		aes_key[i++] = 0xd4;
		aes_key[i++] = 0xb7;
		aes_key[i++] = 0x00;
		aes_key[i++] = 0x32;

		aes_key[i++] = 0x92;
		aes_key[i++] = 0x19;
		aes_key[i++] = 0xbf;
		aes_key[i++] = 0x56;

		aes_key[i++] = 0x89;
		aes_key[i++] = 0xa9;
		aes_key[i++] = 0xcc;
		aes_key[i++] = 0xee;
		sx1231h_set_encryption_key(dev1, aes_key);
		//sx1231h_set_encryption_key(dev2, aes_key);

		//Enable AES
		sx1231h_set_encryption_state(dev1, true);
		sx1231h_set_encryption_state(dev2, true);
	}else if (identifier == TEST_STRINGS_NODE_ADDRESS){

		sendSize = 31;
		sendBuff[0] = 0x21;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);
	}else if (identifier == TEST_STRINGS_BROADCAST_ADDRESS){
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODEBROADCAST, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODEBROADCAST, 0x21, 0x34);
	}else if (identifier == TEST_STRINGS_RX_TIMEOUT_HIGH){
		sx1231h_set_timeout(dev2,255,0);
	}
	delayms(100);

	sx1231h_receiveFrameStart(dev2);
	result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
	if (result == ERROR){
		return ERROR_STRING_TIMEOUT_DEVICE_1;
	}

	rssi = sx1231h_read_rssi(dev2,false);
	result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
	if (result == ERROR){
		return ERROR_STRING_TIMEOUT;
	}else if (result == RX_TIMEOUT){
		return ERROR_STRING_RX_TIMEOUT;
	}

	result = sx1231h_sendFrameWait(dev1);
	if (result == ERROR){
		return ERROR_STRING_TX_TIMEOUT;
	}

	if (receiveSize != sendSize){
		return ERROR_STRING_RX_SIZE_DIFFERENT;
	}

	for(i = 0; i < receiveSize; i++){
		if (sendBuff[i] != receiveBuff[i]){
			if (identifier == TEST_STRINGS_AES_DIFF){
				counter++;
			}else{
				return ERROR_STRING_CORRUPTED_BYTES;
			}
		}
	}
	if (identifier == TEST_STRINGS_AES_DIFF && counter < 14){
		return ERROR_STRING_AES_STRINGS_TOO_SIMILIAR;
	}
	sx1231h_test_identifier(identifier, OMIT_CRLF);
	sprintf((char *)sendBuff, " RSSI %+02d\r\n",rssi);
	uart_OutString((char *)sendBuff);
	return OK;
}

uint8_t sx1231h_test_failure(test_strings_t identifier, uint8_t dev1, uint8_t dev2){
	uint8_t sendBuff[30], receiveBuff[30];
	uint8_t sendSize, receiveSize;
	uint8_t result;
	uint8_t i;

	result = sx1231h_init(dev1);
	if (result == ERROR){
		return ERROR_STRING_DEVICE_1_NOT_DETECTED;
	}
	result = sx1231h_init(dev2);
	if (result == ERROR){
		return ERROR_STRING_DEVICE_2_NOT_DETECTED;
	}
	sendSize = 30;
	for(i = 0; i < sendSize; i++){
		sendBuff[i] = i;
	}

	if (identifier == TEST_STRINGS_DIFF_SYNC){
		spi_write_register_cs(dev1, REG_SYNCVALUE4 | 0x80, 0x95);  //Byte 4 typically is 0x96
	}else if (identifier == TEST_STRINGS_NODE_WRONG_ADDRESS){
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);
	}else if (identifier == TEST_STRINGS_BROADCAST_NOT_ENABLED){
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);
	}else if (identifier == TEST_STRINGS_RX_TIMEOUT_FAILURE){
		sx1231h_set_timeout(dev2,0,1);
	}

	delayms(100);

	sx1231h_receiveFrameStart(dev2);
	result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
	if (result == ERROR){
		return ERROR_STRING_TIMEOUT_DEVICE_1;
	}

	result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
	if (result == OK){
		return ERROR_STRING_SHOULD_HAVE_FAILED;
	}else if (result == RX_TIMEOUT){
		if (identifier == TEST_STRINGS_RX_TIMEOUT_FAILURE){
			return OK;
		}
		return ERROR_STRING_RX_TIMEOUT;
	}
	result = sx1231h_sendFrameWait(dev1);

	return OK;
}

uint8_t sx1231h_test_rssi(test_strings_t identifier, uint8_t dev1, uint8_t dev2){
	uint8_t sendBuff[40], receiveBuff[40];
	uint8_t sendSize, receiveSize;
	uint8_t result;
	uint8_t i;
	uint8_t rssi_test;
	int8_t rssi = 0;
	uint8_t retry = 0;
	for (rssi_test = 0; rssi_test < 180; rssi_test++){
		result = sx1231h_init(dev1);
		if (result == ERROR){
			return ERROR_STRING_DEVICE_1_NOT_DETECTED;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			return ERROR_STRING_DEVICE_2_NOT_DETECTED;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		//iterate down to minimum transmisison level.
		spi_write_register_cs(dev2, REG_RSSITHRESH | 0x80, (255-rssi_test));


		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			return ERROR_STRING_TIMEOUT_DEVICE_1;
		}

		rssi = sx1231h_read_rssi(dev2,false);
		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			retry++;
			if(retry > 10){
				return OK;
			}
			sx1231h_test_identifier(identifier, OMIT_CRLF);
			sprintf((char *)sendBuff, " Timeout Retry\r\n");
			uart_OutString((char *)sendBuff);
			rssi_test--;
			continue;


		}
		retry = 0;

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			return ERROR_STRING_TX_TIMEOUT;
		}

		if (receiveSize != sendSize){
			return ERROR_STRING_RX_SIZE_DIFFERENT;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				return ERROR_STRING_CORRUPTED_BYTES;
			}
		}

		sx1231h_test_identifier(identifier, OMIT_CRLF);
		sprintf((char *)sendBuff, " RSSI %+02d at %+03ddB\r\n",rssi, -((255-rssi_test)>>1));
		uart_OutString((char *)sendBuff);
	}
	return OK;
}

uint8_t sx1231h_test_freq_hop(test_strings_t identifier, uint8_t dev1, uint8_t dev2){
	uint8_t sendBuff[70], receiveBuff[60];
	uint8_t sendSize, receiveSize;
	uint8_t result;
	uint8_t i;
	uint16_t freq_test;
	int8_t rssi = 0;
	uint8_t retry = 0;
	for (freq_test = 870; freq_test < 930; freq_test++){

		result = sx1231h_init(dev1);
		if (result == ERROR){
			return ERROR_STRING_DEVICE_1_NOT_DETECTED;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			return ERROR_STRING_DEVICE_2_NOT_DETECTED;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		sx1231h_change_frequency(dev1,freq_test);
		if (identifier != TEST_STRINGS_FREQ_HOP_FAILURE){
			sx1231h_change_frequency(dev2,freq_test);
		}
		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			return ERROR_STRING_TIMEOUT_DEVICE_1;
		}

		rssi = sx1231h_read_rssi(dev2,false);
		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			retry++;
			if(retry > 10){
				return ERROR_STRING_TIMEOUT;
			}
			sx1231h_test_identifier(identifier, OMIT_CRLF);
			sprintf((char *)sendBuff, " Timeout Retry\r\n");
			uart_OutString((char *)sendBuff);
			freq_test--;
			continue;
		}

		retry = 0;

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			return ERROR_STRING_TX_TIMEOUT;
		}

		if (receiveSize != sendSize){
			return ERROR_STRING_RX_SIZE_DIFFERENT;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				return ERROR_STRING_CORRUPTED_BYTES;
			}
		}

		sx1231h_test_identifier(identifier, OMIT_CRLF);
		sprintf((char *)sendBuff, " RSSI %+02d at Freq %dMHz\r\n",rssi, freq_test);
		uart_OutString((char *)sendBuff);
	}
	return OK;
}

//TODO Test RFM69 with different settings
uint8_t sx1231h_test_wirelessTesting(uint8_t dev1, uint8_t dev2){
	uint8_t result = 0;
	uint8_t tests = 0;
	uart_OutString("Wireless Testing for SX1231H aka RFM69HW\r\n");

	//Test Presence
	for (tests = TEST_STRINGS_RSSI_THRESHOLD; tests < TEST_STRINGS_VOID; tests++){
		result = sx1231h_test(tests,dev1,dev2);
		if (result == OK){
			sx1231h_test_identifier(tests, OMIT_CRLF);
			sx1231h_common_strings(COMMON_STRINGS_TEST_SUCCESS, PRINT_CRLF);
		}else{
			sx1231h_test_identifier(tests, OMIT_CRLF);
			sx1231h_error_strings(result,PRINT_CRLF);
			//sx1231h_dump_select_regs();
		}
	}
	return 0;

	//Test RSSI min levels
	//	{
	//		uint8_t sendBuff[40], receiveBuff[40];
	//		uint8_t sendSize, receiveSize;
	//		uint8_t result;
	//		uint8_t i;
	//		uint8_t rssi_test;
	//		int8_t rssi = 0;
	//		for (rssi_test = 0; rssi_test < 0; rssi_test++){
	//
	//
	//			result = sx1231h_init(dev1);
	//			if (result == ERROR){
	//				uart_OutString("RSSI1 test failed Device 1 not detected\r\n");
	//				sx1231h_dump_select_regs();
	//				return ERROR;
	//			}
	//			result = sx1231h_init(dev2);
	//			if (result == ERROR){
	//				uart_OutString("RSSI1 test failed Device 2 not detected\r\n");
	//				sx1231h_dump_select_regs();
	//				return ERROR;
	//			}
	//			sendSize = 30;
	//			for(i = 0; i < sendSize; i++){
	//				sendBuff[i] = i;
	//			}
	//
	//			//iterate down to minimum transmisison level.
	//			spi_write_register_cs(dev2, REG_RSSITHRESH | 0x80, (255-rssi_test));
	//
	//			delayms(100);
	//
	//			sx1231h_receiveFrameStart(dev2);
	//			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
	//			if (result == ERROR){
	//				uart_OutString("RSSI1 test failed due to timeout for dev1\r\n");
	//				sx1231h_dump_select_regs();
	//				return ERROR;
	//			}
	//			rssi = sx1231h_read_rssi(dev2,false);
	//			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
	//			if (result == ERROR){
	//				//uart_OutString("RSSI1 test failed due to timeout.\r\n");
	//				break;
	//				//sx1231h_dump_select_regs();
	//				//return ERROR;
	//			}
	//
	//			result = sx1231h_sendFrameWait(dev1);
	//			if (result == ERROR){
	//				uart_OutString("RSSI1 test failed due to TX timeout.\r\n");
	//				sx1231h_dump_select_regs();
	//				return ERROR;
	//			}
	//
	//			if (receiveSize != sendSize){
	//				uart_OutString("RSSI1 test failed RX size diff.\r\n");
	//				sx1231h_dump_select_regs();
	//				return ERROR;
	//			}
	//
	//			for(i = 0; i < receiveSize; i++){
	//				if (sendBuff[i] != receiveBuff[i]){
	//					uart_OutString("RSSI1 test failed corrupted bytes in payload\r\n");
	//					sx1231h_dump_select_regs();
	//					return ERROR;
	//				}
	//			}
	//			sprintf((char *)sendBuff, "  RSSI level %+02d, %+02d\r\n",rssi, (255-rssi_test));
	//			uart_OutString((char *)sendBuff);
	//		}
	//		sprintf((char *)sendBuff, "RSSI1 test level %+02d, %+02d\r\n", rssi, (255-rssi_test));
	//		uart_OutString((char *)sendBuff);
	//	}

	//Test RF Frequency Hopping
//	{
//		uint8_t sendBuff[70], receiveBuff[60];
//		uint8_t sendSize, receiveSize;
//		uint8_t result;
//		uint8_t i;
//		uint16_t freq_test;
//		int8_t rssi = 0;
//		uint8_t iterations;
//		for (freq_test = 870; freq_test < 870; freq_test++){ //max 930
//			for(iterations = 0; iterations < 0; iterations++){
//
//				result = sx1231h_init(dev1);
//				if (result == ERROR){
//					uart_OutString("HOP1 test failed Device 1 not detected\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				result = sx1231h_init(dev2);
//				if (result == ERROR){
//					uart_OutString("HOP1 test failed Device 2 not detected\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				sendSize = 30;
//				for(i = 0; i < sendSize; i++){
//					sendBuff[i] = i;
//				}
//
//				sx1231h_change_frequency(dev1,freq_test);
//				sx1231h_change_frequency(dev2,freq_test);
//				delayms(100);
//
//				sx1231h_receiveFrameStart(dev2);
//				result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
//				if (result == ERROR){
//					uart_OutString("HOP1 test failed due to timeout for dev1\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				rssi = sx1231h_read_rssi(dev2,false);
//				result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
//				if (result == ERROR){
//					uart_OutString("HOP1 test failed due to timeout.\r\n");
//					//sx1231h_dump_select_regs();
//
//					//sprintf(sendBuff, "  FAIL: Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
//					//uart_OutString(sendBuff);
//					//return ERROR;
//					//break;
//					continue;
//
//				}
//
//				result = sx1231h_sendFrameWait(dev1);
//				if (result == ERROR){
//					uart_OutString("HOP1 test failed due to TX timeout.\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//
//				if (receiveSize != sendSize){
//					uart_OutString("HOP1 test failed RX size diff.\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//
//				for(i = 0; i < receiveSize; i++){
//					if (sendBuff[i] != receiveBuff[i]){
//						uart_OutString("HOP1 test failed corrupted bytes in payload\r\n");
//						sx1231h_dump_select_regs();
//						return ERROR;
//					}
//				}
//				sprintf((char *)sendBuff, "  Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
//				uart_OutString((char *)sendBuff);
//			}
//			sprintf((char *)sendBuff, "  Freq Hopping Test, %d %+02d, %03d\r\n",iterations, rssi, freq_test);
//			uart_OutString((char *)sendBuff);
//		}
//		sprintf((char *)sendBuff, "HOP1 test level %+02d, %+02d\r\n",rssi, freq_test);
//		uart_OutString((char *)sendBuff);
//	}
//
//	{
//		uint8_t sendBuff[70], receiveBuff[60];
//		uint8_t sendSize, receiveSize;
//		uint8_t result;
//		uint8_t i;
//		uint16_t freq_test;
//		int8_t rssi = 0;
//		uint8_t iterations;
//		for (freq_test = 870; freq_test < 870; freq_test++){
//			for(iterations = 0; iterations < 0; iterations++){
//
//				result = sx1231h_init(dev1);
//				if (result == ERROR){
//					uart_OutString("HOP2 test failed Device 1 not detected\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				result = sx1231h_init(dev2);
//				if (result == ERROR){
//					uart_OutString("HOP2 test failed Device 2 not detected\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				sendSize = 30;
//				for(i = 0; i < sendSize; i++){
//					sendBuff[i] = i;
//				}
//
//				sx1231h_change_frequency(dev1,freq_test);
//				//sx1231h_change_frequency(dev2,freq_test);  //default is 915MHz
//				delayms(100);
//
//				sx1231h_receiveFrameStart(dev2);
//				result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
//				if (result == ERROR){
//					uart_OutString("HOP2 test failed due to timeout for dev1\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//				rssi = sx1231h_read_rssi(dev2,false);
//				result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
//				if (result == ERROR){
//					uart_OutString("HOP2 test failed due to timeout.\r\n");
//					//sx1231h_dump_select_regs();
//
//					//sprintf(sendBuff, "  FAIL: Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
//					//uart_OutString(sendBuff);
//					//return ERROR;
//					//break;
//					continue;
//
//				}
//
//				result = sx1231h_sendFrameWait(dev1);
//				if (result == ERROR){
//					uart_OutString("HOP2 test failed due to TX timeout.\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//
//				if (receiveSize != sendSize){
//					uart_OutString("HOP2 test failed RX size diff.\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//
//				for(i = 0; i < receiveSize; i++){
//					if (sendBuff[i] != receiveBuff[i]){
//						uart_OutString("HOP1 test failed corrupted bytes in payload\r\n");
//						sx1231h_dump_select_regs();
//						return ERROR;
//					}
//				}
//				sprintf((char *)sendBuff, "  Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
//				uart_OutString((char *)sendBuff);
//			}
//			sprintf((char *)sendBuff, "  Freq Hopping Test, %d %+02d, %03d\r\n",iterations, rssi, freq_test);
//			uart_OutString((char *)sendBuff);
//		}
//		sprintf((char *)sendBuff, "HOP2 test level %+02d, %+02d, Passed if only 1 succeeded\r\n",rssi, freq_test);
//		uart_OutString((char *)sendBuff);
//	}

	//listen - Disabled until it is improved.
	/*{
		uint8_t sendBuff[70], receiveBuff[60];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint8_t iterations;
		for(iterations = 0; iterations < 2; iterations++){

			result = sx1231h_init(dev1);
			if (result == ERROR){
				uart_OutString("LIS1 test failed Device 1 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			result = sx1231h_init(dev2);
			if (result == ERROR){
				uart_OutString("LIS1 test failed Device 2 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			sendSize = 30;
			for(i = 0; i < sendSize; i++){
				sendBuff[i] = i;
			}

			delayms(100);

			sx1231h_listenFrameStart(dev2,1000,10000,RF_LISTEN1_CRITERIA_RSSIANDSYNC | RF_LISTEN1_END_SWITCH_TO_MODE,RF_STANDBY);
			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
			if (result == ERROR){
				uart_OutString("LIS1 test failed due to timeout for dev1\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			result = sx1231h_listenFrameWait(dev2, receiveBuff, &receiveSize, RF_STANDBY);
			if (result == ERROR){
				sx1231h_dump_select_regs();
				uart_OutString("LIS1 test failed due to timeout.\r\n");
				uart_OutString((char *)sendBuff);
				return ERROR;
			}

			result = sx1231h_sendFrameWait(dev1);
			if (result == ERROR){
				uart_OutString("LIS1 test failed due to TX timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			if (receiveSize != sendSize){
				uart_OutString("LIS1 test failed RX size diff.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			for(i = 0; i < receiveSize; i++){
				if (sendBuff[i] != receiveBuff[i]){
					uart_OutString("LIS1 test failed corrupted bytes in payload\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
			}
		}
		sprintf((char *)sendBuff, "LIS1 Listen mode successful\r\n");
		uart_OutString((char *)sendBuff);
	} */

	//RX Timeout
//	{
//		uint8_t sendBuff[70], receiveBuff[60];
//		uint8_t sendSize, receiveSize;
//		uint8_t result;
//		uint8_t i;
//		uint8_t iterations;
//		for(iterations = 0; iterations < 2; iterations++){
//
//			result = sx1231h_init(dev1);
//			if (result == ERROR){
//				uart_OutString("TIM1 test failed Device 1 not detected\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//			result = sx1231h_init(dev2);
//			if (result == ERROR){
//				uart_OutString("TIM1 test failed Device 2 not detected\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//			sendSize = 30;
//			for(i = 0; i < sendSize; i++){
//				sendBuff[i] = i;
//			}
//
//			sx1231h_set_timeout(dev2,255,0);
//			delayms(100);
//
//			sx1231h_receiveFrameStart(dev2);
//			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
//			if (result == ERROR){
//				uart_OutString("TIM1 test failed due to timeout for dev1\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//
//			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
//			if (result != OK){
//				sx1231h_dump_select_regs();
//				if (result == 2){
//					uart_OutString("TIM1 test failed due to timeout on rx module\r\n");
//				}else{
//					uart_OutString("TIM1 test failed due to timeout.\r\n");
//				}
//				uart_OutString((char *)sendBuff);
//				return ERROR;
//			}
//
//			result = sx1231h_sendFrameWait(dev1);
//			if (result == ERROR){
//				uart_OutString("TIM1 test failed due to TX timeout.\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//
//			if (receiveSize != sendSize){
//				uart_OutString("TIM1 test failed RX size diff.\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//
//			for(i = 0; i < receiveSize; i++){
//				if (sendBuff[i] != receiveBuff[i]){
//					uart_OutString("TIM1 test failed corrupted bytes in payload\r\n");
//					sx1231h_dump_select_regs();
//					return ERROR;
//				}
//			}
//		}
//		sprintf((char *)sendBuff, "TIM1 Timeout RX mode successful\r\n");
//		uart_OutString((char *)sendBuff);
//	}
//
//	//RX Timeout
//	{
//		uint8_t sendBuff[70], receiveBuff[60];
//		uint8_t sendSize, receiveSize;
//		uint8_t result;
//		uint8_t i;
//		uint8_t iterations;
//		for(iterations = 0; iterations < 2; iterations++){
//
//			result = sx1231h_init(dev1);
//			if (result == ERROR){
//				uart_OutString("TIM2 test failed Device 1 not detected\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//			result = sx1231h_init(dev2);
//			if (result == ERROR){
//				uart_OutString("TIM2 test failed Device 2 not detected\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//			sendSize = 30;
//			for(i = 0; i < sendSize; i++){
//				sendBuff[i] = i;
//			}
//
//			sx1231h_set_timeout(dev2,0,1);
//			delayms(100);
//
//			sx1231h_receiveFrameStart(dev2);
//			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
//			if (result == ERROR){
//				uart_OutString("TIM2 test failed due to timeout for dev1\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//
//			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
//			if (result != OK){
//				//sx1231h_dump_select_regs();
//				if (result == 2){
//					//uart_OutString("TIM2 test succeeded due to timeout on rx module\r\n");
//				}else{
//					sx1231h_dump_select_regs();
//					uart_OutString("TIM2 test failed due to timeout.\r\n");
//					uart_OutString((char *)sendBuff);
//					return ERROR;
//				}
//
//			}else{
//				uart_OutString("TIM2 test failed due to successfully received packet.\r\n");
//				uart_OutString((char *)sendBuff);
//				return ERROR;
//			}
//
//			result = sx1231h_sendFrameWait(dev1);
//			if (result == ERROR){
//				uart_OutString("TIM2 test failed due to TX timeout.\r\n");
//				sx1231h_dump_select_regs();
//				return ERROR;
//			}
//		}
//		sprintf((char *)sendBuff, "TIM2 Timeout RX mode successful\r\n");
//		uart_OutString((char *)sendBuff);
//	}

	//Test TX and RX Large Frame
	//Disabled until the logic improves to become working.//Disabled until logic discovered to get it to work.
	/*{
		uint8_t sendBuff[260], receiveBuff[260];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint16_t txIndex, rxIndex;
		uint16_t timeout = RF_TIMEOUT_RX_WAIT*6;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("LRG1 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("LRG1 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 250;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		delayms(100);

		txIndex = rxIndex = 0;
		sx1231h_receiveLargeFrameStart(dev2);
		result = sx1231h_sendLargeFrameStart(dev1, sendBuff, sendSize, &txIndex);
		if (result == ERROR){
			uart_OutString("LRG1 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		while (timeout-- > 1){
			delayus(10);
			result = sx1231h_receiveLargeFramePoll(dev2, receiveBuff, &receiveSize, &rxIndex);
			if (result == ERROR){
				uart_OutString("LRG1 test failed due to timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			if (result == OK){
				break;
			}
			result = sx1231h_sendLargeFramePoll(dev1,sendBuff, sendSize, &txIndex);
			if (result == ERROR){
				uart_OutString("LRG1 test failed due to TX timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

		}
		if (!timeout){
			uart_OutString("LRG1 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("LRG1 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("LRG1 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
	}*/


	//sx1231h_find_lowest_settings(dev1, dev2);

	return OK;
}
