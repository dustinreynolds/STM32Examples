/*
 * sx1231h.c
 *  The RFM69HW uses a Semtech SX1231H module but handles all of the RF board layout
 *
 *
 *  Created on: May 13, 2015
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


/*******************************************************************
 ** Global variables                                               **
 *******************************************************************/
static   uint32_t RFFrameTimeOut = RF_FRAME_TIMEOUT(1200); // Reception counter value (full frame timeout generation)

uint16_t RegistersCfg[] = { // SX1231 configuration registers values
		DEF_FIFO, // Left for convenience, not to be changed
		DEF_OPMODE | RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY,
		DEF_DATAMODUL | RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00,
		DEF_BITRATEMSB | RF_BITRATEMSB_150000, //By comparing Fdev to Bitrate, a number in the good range was chosen
		DEF_BITRATELSB | RF_BITRATELSB_150000,
		DEF_FDEVMSB | RF_FDEVMSB_150000,
		DEF_FDEVLSB | RF_FDEVLSB_150000,
		DEF_FRFMSB | RF_FRFMSB_915,
		DEF_FRFMID | RF_FRFMID_915,
		DEF_FRFLSB | RF_FRFLSB_915,
		DEF_OSC1,
		DEF_AFCCTRL | RF_AFC_CTRL_STANDARD,
		DEF_LOWBAT | RF_LOWBAT_OFF | RF_LOWBAT_TRIM_1835,
		DEF_LISTEN1 | RF_LISTEN1_RESOL_4100 | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_01,
		DEF_LISTEN2 | RF_LISTEN2_COEFIDLE_VALUE,
		DEF_LISTEN3 | RF_LISTEN3_COEFRX_VALUE,
		DEF_VERSION, 			// Read Only

		DEF_PALEVEL | RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111,
		DEF_PARAMP | RF_PARAMP_40,
		DEF_OCP | RF_OCP_ON | RF_OCP_TRIM_100,

		DEF_AGCREF | RF_AGCREF_AUTO_ON | RF_AGCREF_LEVEL_MINUS80,
		DEF_AGCTHRESH1 | RF_AGCTHRESH1_SNRMARGIN_101 | RF_AGCTHRESH1_STEP1_16,
		DEF_AGCTHRESH2 | RF_AGCTHRESH2_STEP2_7 | RF_AGCTHRESH2_STEP3_11,
		DEF_AGCTHRESH3 | RF_AGCTHRESH3_STEP4_9 | RF_AGCTHRESH3_STEP5_11,
		DEF_LNA | RF_LNA_ZIN_200 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO,
		DEF_RXBW | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0,
		DEF_AFCBW | RF_AFCBW_DCCFREQAFC_100 | RF_AFCBW_MANTAFC_20 | RF_AFCBW_EXPAFC_3,
		DEF_OOKPEAK | RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000,
		DEF_OOKAVG | RF_OOKAVG_AVERAGETHRESHFILT_10,
		DEF_OOKFIX | RF_OOKFIX_FIXEDTHRESH_VALUE,
		DEF_AFCFEI | RF_AFCFEI_AFCAUTOCLEAR_OFF | RF_AFCFEI_AFCAUTO_OFF,
		DEF_AFCMSB, 			// Read Only
		DEF_AFCLSB, 			// Read Only
		DEF_FEIMSB, 			// Read Only
		DEF_FEILSB, 			// Read Only
		DEF_RSSICONFIG | RF_RSSI_FASTRX_OFF,
		DEF_RSSIVALUE,  		// Read Only

		DEF_DIOMAPPING1,
		DEF_DIOMAPPING2 | RF_DIOMAPPING2_CLKOUT_OFF,
		DEF_IRQFLAGS1,
		DEF_IRQFLAGS2,
		DEF_RSSITHRESH | 228,	// Must be set to (-Sensitivity x 2)
		DEF_RXTIMEOUT1 | RF_RXTIMEOUT1_RXSTART_VALUE,
		DEF_RXTIMEOUT2 | RF_RXTIMEOUT2_RSSITHRESH_VALUE,

		DEF_PREAMBLEMSB | RF_PREAMBLESIZE_MSB_VALUE,
		DEF_PREAMBLELSB | RF_PREAMBLESIZE_LSB_VALUE,
		DEF_SYNCCONFIG | RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_0,
		DEF_SYNCVALUE1 | 0x69,
		DEF_SYNCVALUE2 | 0x81,
		DEF_SYNCVALUE3 | 0x7E,
		DEF_SYNCVALUE4 | 0x96,
		DEF_SYNCVALUE5 | RF_SYNC_BYTE5_VALUE,
		DEF_SYNCVALUE6 | RF_SYNC_BYTE6_VALUE,
		DEF_SYNCVALUE7 | RF_SYNC_BYTE7_VALUE,
		DEF_SYNCVALUE8 | RF_SYNC_BYTE8_VALUE,
		DEF_PACKETCONFIG1 | RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF,
		DEF_PAYLOADLENGTH | 255,
		DEF_NODEADRS | RF_NODEADDRESS_VALUE,
		DEF_BROADCASTADRS | RF_BROADCASTADDRESS_VALUE,
		DEF_AUTOMODES | RF_AUTOMODES_ENTER_OFF | RF_AUTOMODES_EXIT_OFF | RF_AUTOMODES_INTERMEDIATE_SLEEP,
		DEF_FIFOTHRESH | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE,
		DEF_PACKETCONFIG2 | RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_OFF | RF_PACKET2_AES_OFF,
		DEF_AESKEY1 | RF_AESKEY1_VALUE,
		DEF_AESKEY2 | RF_AESKEY2_VALUE,
		DEF_AESKEY3 | RF_AESKEY3_VALUE,
		DEF_AESKEY4 | RF_AESKEY4_VALUE,
		DEF_AESKEY5 | RF_AESKEY5_VALUE,
		DEF_AESKEY6 | RF_AESKEY6_VALUE,
		DEF_AESKEY7 | RF_AESKEY7_VALUE,
		DEF_AESKEY8 | RF_AESKEY8_VALUE,
		DEF_AESKEY9 | RF_AESKEY9_VALUE,
		DEF_AESKEY10 | RF_AESKEY10_VALUE,
		DEF_AESKEY11 | RF_AESKEY11_VALUE,
		DEF_AESKEY12 | RF_AESKEY12_VALUE,
		DEF_AESKEY13 | RF_AESKEY13_VALUE,
		DEF_AESKEY14 | RF_AESKEY14_VALUE,
		DEF_AESKEY15 | RF_AESKEY15_VALUE,
		DEF_AESKEY16 | RF_AESKEY16_VALUE,

		DEF_TEMP1 | RF_TEMP1_ADCLOWPOWER_ON,
		DEF_TEMP2
};



void sx1231h_setRFMode(uint8_t spi_device_num, uint8_t mode){
	if (mode == RF_TRANSMITTER){
		spi_write_register_cs(spi_device_num, REG_TESTLNA | 0x80,  RF_RX_SENSITIVITY_BOOST_OFF);
		//spi_write_register_cs(spi_device_num, REG_LNA | 0x80,  RF_LNA_ZIN_50 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO);
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_TRANSMITTER);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == RF_RECEIVER){
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);
		//spi_write_register_cs(spi_device_num, REG_LNA | 0x80,  RF_LNA_ZIN_200 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO);
		//spi_write_register_cs(spi_device_num, REG_TESTLNA | 0x80,  RF_RX_SENSITIVITY_BOOST_ON);
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_RECEIVER);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == RF_SYNTHESIZER){
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SYNTHESIZER);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == RF_STANDBY){
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_STANDBY);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == RF_SLEEP){
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_ABORT_LISTEN_SLEEP);
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SLEEP);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == (RF_LISTEN_THEN_TX)){
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_LISTEN | RF_TRANSMITTER);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == (RF_LISTEN_THEN_RX)){
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);

		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_LISTEN | RF_RECEIVER);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == (RF_LISTEN_THEN_SLEEP)){
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);

		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_LISTEN | RF_SLEEP);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == (RF_LISTEN_THEN_STANDBY)){
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);

		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_LISTEN | RF_STANDBY);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}else if (mode == RF_ABORT_LISTEN_SLEEP){
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0x83) | RF_ABORT_LISTEN_SLEEP);
		spi_write_register_cs(spi_device_num, REG_OPMODE | 0x80, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SLEEP);
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	}

}

uint8_t sx1231h_init(uint8_t spi_device_num){
	uint16_t i;

	i = spi_read_register_cs(spi_device_num, REG_VERSION);
	if ((i== 0x24)||(i==0x23)){
		//uart_OutString("SPI SX1231 Success\r\n");

		//Initialize registers
		for (i = REG_OPMODE; i <= REG_TEMP2; i++){
			spi_write_register_cs(spi_device_num, (i | 0x80), RegistersCfg[i]);
		}

		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);

		RFFrameTimeOut = RF_FRAME_TIMEOUT(38400); // Worst case bitrate

		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
	}else{
		uart_OutString("SPI SX1231 Failure\r\n");
		return 1;
	}
	return 0;
}

uint8_t sx1231h_present(uint8_t spi_device_num){
	uint16_t i;

	i = spi_read_register_cs(spi_device_num, REG_VERSION);

	if (i== 0x24)
		return 1;
	return 0;
}

uint8_t sx1231h_dump_reg(uint8_t spi_device_num){
	uint16_t i = 0;

	for(i=0;i<=0x4f;i++){
		char buffer[40];
		sprintf(buffer, "SX1231h Reg %02x = %02x, init cfg= %02x\r\n",i,spi_read_register_cs(spi_device_num, i), RegistersCfg[i]);
		uart_OutString(buffer);
	}
	return 0;
}

void sx1231h_dump_select_regs(void){
	char buffer[50];
	uint8_t i;

	uart_OutString("REG  Dev1  Dev2  Init\r\n");
	for(i=0;i<=0x71;i++){
		sprintf(buffer, "%02x  %02x  %02x  %02x\r\n",i,spi_read_register_cs(SPI_RFM69_1, i), spi_read_register_cs(SPI_RFM69_2, i), RegistersCfg[i]);
		uart_OutString(buffer);
	}
}

void sx1231h_dump_fifo(uint8_t dev, uint8_t length){
	char buffer[50];
	uint8_t i;

	uart_OutString("Dump Fifo: REG  Dev\r\n");
	for(i=0;i<length;i++){
		sprintf(buffer, "%02x  %02x\r\n",i,spi_read_register_cs(dev, REG_FIFO));
		uart_OutString(buffer);
	}
}

uint8_t sx1231h_sendFrameStart(uint8_t spi_device_num, uint8_t *buffer, uint8_t size){
	uint8_t counter = 0;
	if (size > RF_BUFFER_SIZE_MAX){
		return ERROR;
	}

	spi_write_register_cs(spi_device_num, REG_FIFOTHRESH | 0x80, (RegistersCfg[REG_FIFOTHRESH] & 0x7F) | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);

	sx1231h_setRFMode(spi_device_num, RF_SLEEP);

	spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, size);

	for (counter = 0; counter < size; counter++){
		spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, buffer[counter]);
	}

	sx1231h_setRFMode(spi_device_num, RF_TRANSMITTER);
	return OK;
}

uint8_t sx1231h_sendFrameWait(uint8_t spi_device_num){
	uint16_t timeout = RF_TIMEOUT_WAIT;
	//wait for tx finished
	while (timeout-- > 1){ // Wait for ModeReady
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != 0x00){
			break;
		}
	}
	if (!timeout){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return ERROR;
	}
	sx1231h_setRFMode(spi_device_num, RF_SLEEP);
	return OK;
}


uint8_t sx1231h_sendFrame(uint8_t spi_device_num, uint8_t *buffer, uint8_t size){
	uint8_t counter = 0;
	uint16_t timeout = RF_TIMEOUT_WAIT;
	if ((size + 1) > RF_BUFFER_SIZE_MAX){
		return ERROR;
	}

	spi_write_register_cs(spi_device_num, REG_FIFOTHRESH | 0x80, (RegistersCfg[REG_FIFOTHRESH] & 0x7F) | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);

	sx1231h_setRFMode(spi_device_num, RF_SLEEP);

	spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, size);

	for (counter = 0; counter < size; counter++){
		spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, buffer[counter]);
	}

	sx1231h_setRFMode(spi_device_num, RF_TRANSMITTER);

	//wait for tx finished
	while (timeout-- > 1){ // Wait for ModeReady
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != 0x00){
			break;
		}
	}
	if (!timeout){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return ERROR;
	}
	sx1231h_setRFMode(spi_device_num, RF_SLEEP);
	return OK;
}

uint8_t sx1231h_receiveFrameStart(uint8_t spi_device_num){
	//avoid starting RX when packet ready to download
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG2 | 0x80, (spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & 0xFB) | RF_PACKET2_RXRESTART);
	}

	spi_write_register_cs(spi_device_num, REG_SYNCCONFIG | 0x80, (RegistersCfg[REG_SYNCCONFIG] & 0xBF) | RF_SYNC_FIFOFILL_AUTO);

	sx1231h_setRFMode(spi_device_num, RF_RECEIVER);
	return 0;
}

uint8_t sx1231h_receiveFrameWait(uint8_t spi_device_num, uint8_t *buffer, uint8_t * size){
	uint16_t timeout = RF_TIMEOUT_RX_WAIT;
	uint8_t i;
	while (timeout-- > 1){
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
			break;
		}
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT) != 0x00){
			sx1231h_setRFMode(spi_device_num, RF_SLEEP);
			return 2;
		}
		delayus(10);
	}
	if (!timeout){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return ERROR;
	}

	//TODO eventually setup timeout reg on wireless module to generate a timeout interrupt

	//Read payload
	sx1231h_setRFMode(spi_device_num, RF_SLEEP);
	*size = spi_read_register_cs(spi_device_num, REG_FIFO);

	for(i=0; i < size[0]; i++){
		*buffer++ = spi_read_register_cs(spi_device_num, REG_FIFO);
	}
	return OK;
}

uint8_t sx1231h_receiveFrame(uint8_t spi_device_num,uint8_t *buffer, uint8_t * size){
	uint16_t timeout = RF_TIMEOUT_WAIT;
	uint8_t i;

	//avoid starting RX when packet ready to download
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG2 | 0x80, (spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & 0xFB) | RF_PACKET2_RXRESTART);
	}

	sx1231h_setRFMode(spi_device_num, RF_RECEIVER);

	while (timeout-- > 0){
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
			break;
		}
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT) != 0x00){
			sx1231h_setRFMode(spi_device_num, RF_SLEEP);
			return 1;
		}
		delayms(10); //300 x 10ms = 3 sec
	}
	if (!timeout){
		return ERROR;
	}

	//eventually setup timeout reg on wireless module to generate a timeout interrupt

	//Read payload
	sx1231h_setRFMode(spi_device_num, RF_SLEEP);
	*size = spi_read_register_cs(spi_device_num, REG_FIFO);

	for(i=0; i < size[0]; i++){
		*buffer++ = spi_read_register_cs(spi_device_num, REG_FIFO);
	}
	return 0;
}

//can listen/rx from 64us to a maximum of 66.81 seconds
//Can't handle ListenEnd = 10 yet
uint8_t sx1231h_listenFrameStart(uint8_t spi_device_num, uint32_t idleus, uint32_t rxus, uint8_t listenCtl, uint8_t nextMode){
	uint32_t usecs;
	uint8_t listen1 = 0;
	uint8_t listen2 = 0;
	uint8_t listen3 = 0;
	return ERROR; //This mode seems a bit broken.
	//avoid starting RX when packet ready to download
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG2 | 0x80, (spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & 0xFB) | RF_PACKET2_RXRESTART);
	}

	if ((listenCtl & 0x0E) != listenCtl){
		//Extra bytes are specified, return
		return 0;
	}
	listen1 = (listenCtl & 0x0E);

	if (idleus <= 16320){
		listen1 |= RF_LISTEN1_RESOLIDLE_64;
		usecs = idleus / 64;
	}else if (idleus > 16320 && idleus <= 1045500){
		listen1 |= RF_LISTEN1_RESOLIDLE_4100;
		usecs = idleus / 4100;
	}else if (idleus > 1045500 ){
		listen1 |= RF_LISTEN1_RESOLIDLE_262000;
		usecs = idleus / 262000;
	}
	listen2 = usecs & 0xFF;

	if (rxus <= 16320){
		listen1 |= RF_LISTEN1_RESOLRX_64;
		usecs = rxus / 64;
	}else if (rxus > 16320 && idleus <= 1045500){
		listen1 |= RF_LISTEN1_RESOLRX_4100;
		usecs = rxus / 4100;
	}else if (rxus > 1045500 ){
		listen1 |= RF_LISTEN1_RESOLRX_262000;
		usecs = rxus / 262000;
	}
	listen3 = usecs & 0xFF;

	sx1231h_setRFMode(spi_device_num, RF_STANDBY);

	//set listenIdle time, listen rx time,
	spi_write_register_cs(spi_device_num, REG_LISTEN1 | 0x80, listen1);
	spi_write_register_cs(spi_device_num, REG_LISTEN2 | 0x80, listen2);
	spi_write_register_cs(spi_device_num, REG_LISTEN3 | 0x80, listen3);
	sx1231h_setRFMode(spi_device_num, RF_LISTEN | nextMode);
	return 0;
}

uint8_t sx1231h_listenFrameWait(uint8_t spi_device_num, uint8_t *buffer, uint8_t * size, uint8_t mode){
	uint16_t timeout = RF_TIMEOUT_RX_WAIT;
	uint8_t i;
	return ERROR; //This mode seems a bit broken.
	while (timeout-- > 1){
		if ((spi_read_register_cs(spi_device_num, REG_OPMODE) & 0x1E) == mode){
			break;
		}
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
			break;
		}
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT) != 0x00){
			sx1231h_setRFMode(spi_device_num, RF_ABORT_LISTEN_SLEEP);
			return 2;
		}
		delayus(10);
	}

	sx1231h_setRFMode(spi_device_num, RF_ABORT_LISTEN_SLEEP);
	if (!timeout){
		return ERROR;
	}

	//TODO eventually setup timeout reg on wireless module to generate a timeout interrupt

	//Read payload
	*size = spi_read_register_cs(spi_device_num, REG_FIFO);

	for(i=0; i < size[0]; i++){
		*buffer++ = spi_read_register_cs(spi_device_num, REG_FIFO);
	}
	return OK;
}

//Disabled until logic discovered to get it to work.
uint8_t sx1231h_receiveLargeFrameStart(uint8_t spi_device_num){

	return ERROR;
	//avoid starting RX when packet ready to download
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG2 | 0x80, (spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & 0xFB) | RF_PACKET2_RXRESTART);
	}

	spi_write_register_cs(spi_device_num, REG_SYNCCONFIG | 0x80, (RegistersCfg[REG_SYNCCONFIG] & 0xBF) | RF_SYNC_FIFOFILL_AUTO);

	sx1231h_setRFMode(spi_device_num, RF_RECEIVER);
	return 0;
}

//Disabled until logic discovered to get it to work.
uint8_t sx1231h_receiveLargeFramePoll(uint8_t spi_device_num, uint8_t *buffer, uint8_t * size, uint16_t * index){
	uint16_t timeout = RF_TIMEOUT_RX_WAIT;
	uint8_t i;
	uint8_t subIndex = 0;
	return ERROR;
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return OK;
	}

	while((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) != 0x00){
		if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) != 0x00){
			break;
		}

		if (*index == 0){
			*size = spi_read_register_cs(spi_device_num, REG_FIFO);
		}else{
			buffer[*index-1] = spi_read_register_cs(spi_device_num, REG_FIFO);
		}
		*index += 1;
	}

	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT) != 0x00){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return RX_TIMEOUT;
	}

	if(*index > 1 && *size == *index){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return OK;
	}
	return RX_RUNNING;
}

//Disabled until logic discovered to get it to work.
uint8_t sx1231h_sendLargeFrameStart(uint8_t spi_device_num, uint8_t *buffer, uint8_t size, uint16_t * index){
	uint8_t subIndex = 0;
	return ERROR;
	spi_write_register_cs(spi_device_num, REG_FIFOTHRESH | 0x80, (RegistersCfg[REG_FIFOTHRESH] & 0x7F) | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);

	sx1231h_setRFMode(spi_device_num, RF_SLEEP);

	spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, size);

	while((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOFULL) == 0x00){
		spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, buffer[*index]);
		*index += 1;
		subIndex = 1;
	}

	if (subIndex){
		*index -= 1;
	}
	sx1231h_setRFMode(spi_device_num, RF_TRANSMITTER);
	return OK;
}

//Disabled until logic discovered to get it to work.
uint8_t sx1231h_sendLargeFramePoll(uint8_t spi_device_num, uint8_t *buffer, uint8_t size, uint16_t * index){
	return ERROR;
	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != 0x00){
		sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return OK;
	}

	if (*index > size){
		//sx1231h_setRFMode(spi_device_num, RF_SLEEP);
		return OK;
	}

	if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) != 0x00){
		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOFULL) == 0x00){
			spi_write_register_cs(spi_device_num, REG_FIFO | 0x80, buffer[*index]);

			if (*index > size){
				//sx1231h_setRFMode(spi_device_num, RF_SLEEP);
				return OK;
			}
			*index += 1;
			if((spi_read_register_cs(spi_device_num, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != 0x00){
				sx1231h_setRFMode(spi_device_num, RF_SLEEP);
				return OK;
			}
		}
	}

	return TX_RUNNING;
}


//Note: Encryption key stored on RFM69HW is readable, despite the manual indicating that it is write only
void sx1231h_set_encryption_state(uint8_t spi_device_num, bool aes_state){
	spi_write_register_cs(spi_device_num, REG_PACKETCONFIG2 | 0x80, (spi_read_register_cs(spi_device_num, REG_PACKETCONFIG2) & 0xFE) | aes_state ? RF_PACKET2_AES_ON : RF_PACKET2_AES_OFF);
}

void sx1231h_set_encryption_key(uint8_t spi_device_num, uint8_t * aes_key){
	uint8_t i;

	for (i = 0; i < 16; i++){
		spi_write_register_cs(spi_device_num, ((REG_AESKEY1 + i)| 0x80),aes_key[i]);
	}
}

//only applies to TX
void sx1231h_set_power(uint8_t spi_device_num, int8_t power_level){

	if (power_level < -18){
		power_level = -18;
	}

	if(power_level <= 13){ //PA0 on, RFIO
		spi_write_register_cs(spi_device_num, REG_OCP | 0x80, RF_OCP_ON | RF_OCP_TRIM_45);
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_PALEVEL | 0x80, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | (power_level + RF_PALEVEL_PA0_OFFSET));
	}else if (power_level > 13 && power_level <= 17){ //PA1 and PA2 on, PA_boost
		spi_write_register_cs(spi_device_num, REG_OCP | 0x80, DEF_OCP | RF_OCP_ON | RF_OCP_TRIM_95);
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_OFF);
		spi_write_register_cs(spi_device_num, REG_PALEVEL | 0x80, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | (power_level + RF_PALEVEL_PA1_PA2_OFFSET));
	}else if (power_level > 17 && power_level < 19){ //PA1 and PA2 on, PA_boost with High output power
		spi_write_register_cs(spi_device_num, REG_OCP | 0x80, DEF_OCP | RF_OCP_ON | RF_OCP_TRIM_120);
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_ON);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_ON);
		spi_write_register_cs(spi_device_num, REG_PALEVEL | 0x80, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | (power_level + RF_PALEVEL_PA1_PA2_HIGH_OFFSET));
	}else if (power_level >= 19){
		spi_write_register_cs(spi_device_num, REG_OCP | 0x80, DEF_OCP | RF_OCP_OFF | RF_OCP_TRIM_120);
		spi_write_register_cs(spi_device_num, REG_TESTPA1 | 0x80, RF_TESTPA1_20DBM_ON);
		spi_write_register_cs(spi_device_num, REG_TESTPA2 | 0x80, RF_TESTPA2_20DBM_ON);
		spi_write_register_cs(spi_device_num, REG_PALEVEL | 0x80, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_PALEVEL_OUTPUTPOWER_11111);
	}
}

//need to test this
void sx1231h_set_baudrate(uint8_t spi_device_num, uint32_t baud){
	uint16_t bitrate;

	bitrate = (32000000/baud);
	spi_write_register_cs(spi_device_num, REG_BITRATEMSB | 0x80, (uint8_t)(bitrate >> 8));
	spi_write_register_cs(spi_device_num, REG_BITRATELSB | 0x80, (uint8_t)(bitrate & 0x00FF));
}

void sx1231h_set_freq_dev(uint8_t spi_device_num, uint32_t freq ){
	uint16_t freq_dev = (uint16_t)((float)freq * 0.0164);
	spi_write_register_cs(spi_device_num, REG_FDEVMSB | 0x80, (uint8_t)(freq_dev >> 8));
	spi_write_register_cs(spi_device_num, REG_FDEVLSB | 0x80, (uint8_t)(freq_dev & 0x00FF));
}

int8_t sx1231h_read_rssi(uint8_t spi_device_num, bool trigger){
	uint16_t timeout = RF_TIMEOUT_WAIT;
	if(trigger){
		spi_write_register_cs(spi_device_num, REG_RSSICONFIG | 0x80, RF_RSSI_START);
		while (timeout-- > 1){ // Wait for ModeReady
			if((spi_read_register_cs(spi_device_num, REG_RSSICONFIG) & RF_RSSI_DONE) != 0x00){
				break;
			}
		}
	}
	return -(spi_read_register_cs(spi_device_num, REG_RSSIVALUE) >> 1);
}

void sx1231h_set_addr_filtering(uint8_t spi_device_num, uint8_t state, uint8_t thisAddress, uint8_t broadcastAddress){
	uint8_t reg = spi_read_register_cs(spi_device_num, REG_PACKETCONFIG1);

	if (state == RF_PACKET1_ADRSFILTERING_OFF){
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG1 | 0x80, (reg & 0xF8));
	}else if (state == RF_PACKET1_ADRSFILTERING_NODE){ //Node must match
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG1 | 0x80, (reg & 0xF8) | 0x02);
		spi_write_register_cs(spi_device_num, REG_NODEADRS | 0x80,thisAddress);
		spi_write_register_cs(spi_device_num, REG_BROADCASTADRS | 0x80, broadcastAddress);
	}else if (state == RF_PACKET1_ADRSFILTERING_NODEBROADCAST){ //Node or broadcast must match
		spi_write_register_cs(spi_device_num, REG_PACKETCONFIG1 | 0x80, (reg & 0xF8) | 0x04);
		spi_write_register_cs(spi_device_num, REG_NODEADRS | 0x80,thisAddress);
		spi_write_register_cs(spi_device_num, REG_BROADCASTADRS | 0x80, broadcastAddress);
	}
}

void sx1231h_change_frequency(uint8_t spi_device_num, uint16_t freq){
	sx1231h_setRFMode(spi_device_num, RF_SLEEP);

	if (freq >= 863 && freq < 950){
		uint16_t offset = freq - 863;
		uint32_t freq_value = RF_FRFMSB_863 << 16 | RF_FRFMID_863 << 8 | RF_FRFLSB_863;

		freq_value += 0x4000 * offset;

		spi_write_register_cs(spi_device_num, REG_FRFMSB | 0x80, ((freq_value >> 16) & 0xFF));
		spi_write_register_cs(spi_device_num, REG_FRFMID | 0x80, ((freq_value >> 8) & 0xFF));
		spi_write_register_cs(spi_device_num, REG_FRFLSB | 0x80, (freq_value & 0xFF));

		while ((spi_read_register_cs(spi_device_num, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

	}
}

/*
 * rxTimeout is timeout for first signal detection
 * rssiTimeout is timeout for first signal detection to payload ready
 */
void sx1231h_set_timeout(uint8_t spi_device_num, uint8_t rxTimeout, uint8_t rssiTimeout){
	spi_write_register_cs(spi_device_num, REG_RXTIMEOUT1 | 0x80, rxTimeout);
	spi_write_register_cs(spi_device_num, REG_RXTIMEOUT2 | 0x80, rssiTimeout);
}

//Test different lengths, versus bitrate, versus power, to determine the minimum power needed at each bitrate, to send a given length message
uint8_t sx1231h_find_lowest_settings(uint8_t dev1, uint8_t dev2){
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

//TODO Test RFM69 with different settings
uint8_t sx1231h_wirelessTesting(uint8_t dev1, uint8_t dev2){
	uart_OutString("Wireless Testing for SX1231H aka RFM69HW\r\n");

	//Test Presence
	{
		if(sx1231h_present(dev1) == 0){
			uart_OutString("RX/TX test failed Device 1 not detected\r\n");
			return ERROR;
		}
		if(sx1231h_present(dev2) == 0){
			uart_OutString("RX/TX test failed Device 1 not detected\r\n");
			return ERROR;
		}
	}

	//Test TX and RX
	{
		uint8_t sendBuff[30], receiveBuff[30];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		int8_t rssi;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("RF1 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("RF1 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("RF1 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			uart_OutString("RF1 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		rssi = sx1231h_read_rssi(dev1,false);
		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("RF1 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("RF1 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("RF1 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		uart_OutString("Dev1 TX, Dev2 RX test succeeded. ");
		sprintf((char *)sendBuff, "RSSI %+02d\r\n",rssi);
		uart_OutString((char *)sendBuff);
	}

	//Test RX and TX
	{
		uint8_t sendBuff[30], receiveBuff[30];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("RF2 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("RF2 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		delayms(100);

		sx1231h_receiveFrameStart(dev1);
		result = sx1231h_sendFrameStart(dev2, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("RF2 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev1, receiveBuff, &receiveSize);
		if (result == ERROR){
			uart_OutString("RF2 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev2);
		if (result == ERROR){
			uart_OutString("RF2 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("RF2 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("RF2 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		uart_OutString("Dev2 TX, Dev1 RX test succeeded.\r\n");
	}

	//Test Different Sync bytes
	{
		uint8_t sendBuff[30], receiveBuff[30];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("RF3 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("RF3 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

		spi_write_register_cs(dev1, REG_SYNCVALUE4 | 0x80, 0x95);  //Byte 4 typically is 0x96

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("RF3 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result != ERROR){
			uart_OutString("RF3 test failed due to successfully receiving packet.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("RF3 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		uart_OutString("RF test Different Sync Byte 4 succeeded.\r\n");
	}

	//Test TX and RX with AES ON
	{
		uint8_t sendBuff[30], receiveBuff[30];
		uint8_t aes_key[16];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("AES1 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("AES1 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

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

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("AES1 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			uart_OutString("AES1 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("AES1 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("AES1 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("AES1 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		uart_OutString("RF AES Enabled test succeeded.\r\n");
	}

	//Test TX with AES on and RX with AES off
	{
		uint8_t sendBuff[30], receiveBuff[30];
		uint8_t aes_key[16];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint8_t counter = 0;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("AES2 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("AES2 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 30;
		for(i = 0; i < sendSize; i++){
			sendBuff[i] = i;
		}

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
		aes_key[i++] = 0xee;
		sx1231h_set_encryption_key(dev1, aes_key);
		//sx1231h_set_encryption_key(dev2, aes_key);

		//Enable AES
		sx1231h_set_encryption_state(dev1, true);
		sx1231h_set_encryption_state(dev2, true);

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("AES2 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			//sx1231h_dump_fifo(dev2, 31);
			uart_OutString("AES2 test failed due to RX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("AES2 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("AES2 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				counter++;
			}
		}
		if (counter < 14){
			uart_OutString("AES2 AES test failed packets too similiar!\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		uart_OutString("RF TX AES, RX AES diff key test succeeded.\r\n");
	}

	//Test Address Filtering, Sending packet to this addr.
	{
		uint8_t sendBuff[40], receiveBuff[40];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;


		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("Addr1 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("Addr1 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 31;
		sendBuff[0] = 0x21;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("Addr1 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			uart_OutString("Addr1 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("Addr1 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("Addr1 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("Addr1 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		uart_OutString("Addr1 send just to Node Addr test succeeded.\r\n");
	}

	//Test Address Filtering, Sending packet to broadcast addr.
	{
		uint8_t sendBuff[40], receiveBuff[40];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("Addr2 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("Addr2 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODEBROADCAST, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODEBROADCAST, 0x21, 0x34);

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("Addr2 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result == ERROR){
			uart_OutString("Addr2 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_sendFrameWait(dev1);
		if (result == ERROR){
			uart_OutString("Addr2 test failed due to TX timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		if (receiveSize != sendSize){
			uart_OutString("Addr2 test failed RX size diff.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		for(i = 0; i < receiveSize; i++){
			if (sendBuff[i] != receiveBuff[i]){
				uart_OutString("Addr2 test failed corrupted bytes in payload\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		uart_OutString("Addr2 send to broadcast addr test succeeded.\r\n");
	}

	//test addr filtering, send to different addr
	{
		uint8_t sendBuff[40], receiveBuff[40];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("Addr3 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("Addr3 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("Addr3 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result != ERROR){
			uart_OutString("Addr3 test failed due to not timing out.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		uart_OutString("Addr3 send to other addr test Succeeded.\r\n");
	}

	//Test Address Filtering, Sending packet to broadcast addr without it enabled
	{
		uint8_t sendBuff[40], receiveBuff[40];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;

		result = sx1231h_init(dev1);
		if (result == ERROR){
			uart_OutString("Addr4 test failed Device 1 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		result = sx1231h_init(dev2);
		if (result == ERROR){
			uart_OutString("Addr4 test failed Device 2 not detected\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		sendSize = 31;
		sendBuff[0] = 0x34;
		for(i = 0; i < sendSize; i++){
			sendBuff[i+1] = i;
		}

		sx1231h_set_addr_filtering(dev1, RF_PACKET1_ADRSFILTERING_NODE, 0x12, 0x34);
		sx1231h_set_addr_filtering(dev2, RF_PACKET1_ADRSFILTERING_NODE, 0x21, 0x34);

		delayms(100);

		sx1231h_receiveFrameStart(dev2);
		result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
		if (result == ERROR){
			uart_OutString("Addr4 test failed due to timeout for dev1\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}

		result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
		if (result != ERROR){
			uart_OutString("Addr4 test failed due to timeout.\r\n");
			sx1231h_dump_select_regs();
			return ERROR;
		}
		uart_OutString("Addr4 send to disabled broadcast addr test succeeded.\r\n");
	}

	//Test RSSI min levels
	{
		uint8_t sendBuff[40], receiveBuff[40];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint8_t rssi_test;
		int8_t rssi = 0;
		for (rssi_test = 0; rssi_test < 0; rssi_test++){


			result = sx1231h_init(dev1);
			if (result == ERROR){
				uart_OutString("RSSI1 test failed Device 1 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			result = sx1231h_init(dev2);
			if (result == ERROR){
				uart_OutString("RSSI1 test failed Device 2 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
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
				uart_OutString("RSSI1 test failed due to timeout for dev1\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			rssi = sx1231h_read_rssi(dev2,false);
			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
			if (result == ERROR){
				//uart_OutString("RSSI1 test failed due to timeout.\r\n");
				break;
				//sx1231h_dump_select_regs();
				//return ERROR;
			}

			result = sx1231h_sendFrameWait(dev1);
			if (result == ERROR){
				uart_OutString("RSSI1 test failed due to TX timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			if (receiveSize != sendSize){
				uart_OutString("RSSI1 test failed RX size diff.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			for(i = 0; i < receiveSize; i++){
				if (sendBuff[i] != receiveBuff[i]){
					uart_OutString("RSSI1 test failed corrupted bytes in payload\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
			}
			sprintf((char *)sendBuff, "  RSSI level %+02d, %+02d\r\n",rssi, (255-rssi_test));
			uart_OutString((char *)sendBuff);
		}
		sprintf((char *)sendBuff, "RSSI1 test level %+02d, %+02d\r\n", rssi, (255-rssi_test));
		uart_OutString((char *)sendBuff);
	}

	//Test RF Frequency Hopping
	{
		uint8_t sendBuff[70], receiveBuff[60];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint16_t freq_test;
		int8_t rssi = 0;
		uint8_t iterations;
		for (freq_test = 870; freq_test < 870; freq_test++){ //max 930
			for(iterations = 0; iterations < 0; iterations++){

				result = sx1231h_init(dev1);
				if (result == ERROR){
					uart_OutString("HOP1 test failed Device 1 not detected\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				result = sx1231h_init(dev2);
				if (result == ERROR){
					uart_OutString("HOP1 test failed Device 2 not detected\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				sendSize = 30;
				for(i = 0; i < sendSize; i++){
					sendBuff[i] = i;
				}

				sx1231h_change_frequency(dev1,freq_test);
				sx1231h_change_frequency(dev2,freq_test);
				delayms(100);

				sx1231h_receiveFrameStart(dev2);
				result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
				if (result == ERROR){
					uart_OutString("HOP1 test failed due to timeout for dev1\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				rssi = sx1231h_read_rssi(dev2,false);
				result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
				if (result == ERROR){
					uart_OutString("HOP1 test failed due to timeout.\r\n");
					//sx1231h_dump_select_regs();

					//sprintf(sendBuff, "  FAIL: Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
					//uart_OutString(sendBuff);
					//return ERROR;
					//break;
					continue;

				}

				result = sx1231h_sendFrameWait(dev1);
				if (result == ERROR){
					uart_OutString("HOP1 test failed due to TX timeout.\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}

				if (receiveSize != sendSize){
					uart_OutString("HOP1 test failed RX size diff.\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}

				for(i = 0; i < receiveSize; i++){
					if (sendBuff[i] != receiveBuff[i]){
						uart_OutString("HOP1 test failed corrupted bytes in payload\r\n");
						sx1231h_dump_select_regs();
						return ERROR;
					}
				}
				sprintf((char *)sendBuff, "  Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
				uart_OutString((char *)sendBuff);
			}
			sprintf((char *)sendBuff, "  Freq Hopping Test, %d %+02d, %03d\r\n",iterations, rssi, freq_test);
			uart_OutString((char *)sendBuff);
		}
		sprintf((char *)sendBuff, "HOP1 test level %+02d, %+02d\r\n",rssi, freq_test);
		uart_OutString((char *)sendBuff);
	}

	{
		uint8_t sendBuff[70], receiveBuff[60];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint16_t freq_test;
		int8_t rssi = 0;
		uint8_t iterations;
		for (freq_test = 870; freq_test < 870; freq_test++){
			for(iterations = 0; iterations < 0; iterations++){

				result = sx1231h_init(dev1);
				if (result == ERROR){
					uart_OutString("HOP2 test failed Device 1 not detected\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				result = sx1231h_init(dev2);
				if (result == ERROR){
					uart_OutString("HOP2 test failed Device 2 not detected\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				sendSize = 30;
				for(i = 0; i < sendSize; i++){
					sendBuff[i] = i;
				}

				sx1231h_change_frequency(dev1,freq_test);
				//sx1231h_change_frequency(dev2,freq_test);  //default is 915MHz
				delayms(100);

				sx1231h_receiveFrameStart(dev2);
				result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
				if (result == ERROR){
					uart_OutString("HOP2 test failed due to timeout for dev1\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
				rssi = sx1231h_read_rssi(dev2,false);
				result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
				if (result == ERROR){
					uart_OutString("HOP2 test failed due to timeout.\r\n");
					//sx1231h_dump_select_regs();

					//sprintf(sendBuff, "  FAIL: Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
					//uart_OutString(sendBuff);
					//return ERROR;
					//break;
					continue;

				}

				result = sx1231h_sendFrameWait(dev1);
				if (result == ERROR){
					uart_OutString("HOP2 test failed due to TX timeout.\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}

				if (receiveSize != sendSize){
					uart_OutString("HOP2 test failed RX size diff.\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}

				for(i = 0; i < receiveSize; i++){
					if (sendBuff[i] != receiveBuff[i]){
						uart_OutString("HOP1 test failed corrupted bytes in payload\r\n");
						sx1231h_dump_select_regs();
						return ERROR;
					}
				}
				sprintf((char *)sendBuff, "  Freq Hopping Test, RSSI level %+02d, %+02d\r\n",rssi, freq_test);
				uart_OutString((char *)sendBuff);
			}
			sprintf((char *)sendBuff, "  Freq Hopping Test, %d %+02d, %03d\r\n",iterations, rssi, freq_test);
			uart_OutString((char *)sendBuff);
		}
		sprintf((char *)sendBuff, "HOP2 test level %+02d, %+02d, Passed if only 1 succeeded\r\n",rssi, freq_test);
		uart_OutString((char *)sendBuff);
	}

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
	{
		uint8_t sendBuff[70], receiveBuff[60];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint8_t iterations;
		for(iterations = 0; iterations < 2; iterations++){

			result = sx1231h_init(dev1);
			if (result == ERROR){
				uart_OutString("TIM1 test failed Device 1 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			result = sx1231h_init(dev2);
			if (result == ERROR){
				uart_OutString("TIM1 test failed Device 2 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			sendSize = 30;
			for(i = 0; i < sendSize; i++){
				sendBuff[i] = i;
			}

			sx1231h_set_timeout(dev2,255,0);
			delayms(100);

			sx1231h_receiveFrameStart(dev2);
			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
			if (result == ERROR){
				uart_OutString("TIM1 test failed due to timeout for dev1\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
			if (result != OK){
				sx1231h_dump_select_regs();
				if (result == 2){
					uart_OutString("TIM1 test failed due to timeout on rx module\r\n");
				}else{
					uart_OutString("TIM1 test failed due to timeout.\r\n");
				}
				uart_OutString((char *)sendBuff);
				return ERROR;
			}

			result = sx1231h_sendFrameWait(dev1);
			if (result == ERROR){
				uart_OutString("TIM1 test failed due to TX timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			if (receiveSize != sendSize){
				uart_OutString("TIM1 test failed RX size diff.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			for(i = 0; i < receiveSize; i++){
				if (sendBuff[i] != receiveBuff[i]){
					uart_OutString("TIM1 test failed corrupted bytes in payload\r\n");
					sx1231h_dump_select_regs();
					return ERROR;
				}
			}
		}
		sprintf((char *)sendBuff, "TIM1 Timeout RX mode successful\r\n");
		uart_OutString((char *)sendBuff);
	}

	//RX Timeout
	{
		uint8_t sendBuff[70], receiveBuff[60];
		uint8_t sendSize, receiveSize;
		uint8_t result;
		uint8_t i;
		uint8_t iterations;
		for(iterations = 0; iterations < 2; iterations++){

			result = sx1231h_init(dev1);
			if (result == ERROR){
				uart_OutString("TIM2 test failed Device 1 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			result = sx1231h_init(dev2);
			if (result == ERROR){
				uart_OutString("TIM2 test failed Device 2 not detected\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
			sendSize = 30;
			for(i = 0; i < sendSize; i++){
				sendBuff[i] = i;
			}

			sx1231h_set_timeout(dev2,0,1);
			delayms(100);

			sx1231h_receiveFrameStart(dev2);
			result = sx1231h_sendFrameStart(dev1, sendBuff, sendSize);
			if (result == ERROR){
				uart_OutString("TIM2 test failed due to timeout for dev1\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}

			result = sx1231h_receiveFrameWait(dev2, receiveBuff, &receiveSize);
			if (result != OK){
				//sx1231h_dump_select_regs();
				if (result == 2){
					//uart_OutString("TIM2 test succeeded due to timeout on rx module\r\n");
				}else{
					sx1231h_dump_select_regs();
					uart_OutString("TIM2 test failed due to timeout.\r\n");
					uart_OutString((char *)sendBuff);
					return ERROR;
				}

			}else{
				uart_OutString("TIM2 test failed due to successfully received packet.\r\n");
				uart_OutString((char *)sendBuff);
				return ERROR;
			}

			result = sx1231h_sendFrameWait(dev1);
			if (result == ERROR){
				uart_OutString("TIM2 test failed due to TX timeout.\r\n");
				sx1231h_dump_select_regs();
				return ERROR;
			}
		}
		sprintf((char *)sendBuff, "TIM2 Timeout RX mode successful\r\n");
		uart_OutString((char *)sendBuff);
	}

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
