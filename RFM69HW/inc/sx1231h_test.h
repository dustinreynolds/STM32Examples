/*
 * sx1231h_test.h
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

#ifndef SX1231H_TEST_H_
#define SX1231H_TEST_H_

#define PRINT_CRLF 	1
#define OMIT_CRLF	0

typedef enum {
	TEST_STRINGS_TEST_PRESENCE = 0,
	TEST_STRINGS_BASIC_RX_TX,
	TEST_STRINGS_DIFF_SYNC,
	TEST_STRINGS_AES_ON,
	TEST_STRINGS_AES_DIFF,
	TEST_STRINGS_NODE_ADDRESS,
	TEST_STRINGS_BROADCAST_ADDRESS,
	TEST_STRINGS_NODE_WRONG_ADDRESS,
	TEST_STRINGS_BROADCAST_NOT_ENABLED,
	TEST_STRINGS_RSSI_THRESHOLD,
	TEST_STRINGS_FREQ_HOP,
	TEST_STRINGS_FREQ_HOP_FAILURE,
	TEST_STRINGS_RX_TIMEOUT_HIGH,
	TEST_STRINGS_RX_TIMEOUT_FAILURE,
	TEST_STRINGS_VOID, //Be sure to add new tests to sx1231h_test and sx1231_test_identifier
} test_strings_t;

typedef struct {
	test_strings_t testIdentifier;
	uint8_t (*function)(test_strings_t ,uint8_t, uint8_t);
} pTable_t;

uint8_t sx1231h_test_presence(test_strings_t identifier, uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_test_single_change(test_strings_t identifier, uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_test_failure(test_strings_t identifier, uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_test_rssi(test_strings_t identifier, uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_test_freq_hop(test_strings_t identifier, uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_find_lowest_settings(uint8_t dev1, uint8_t dev2);
uint8_t sx1231h_wirelessTesting(uint8_t dev1, uint8_t dev2);

#endif /* SX1231H_TEST_H_ */
