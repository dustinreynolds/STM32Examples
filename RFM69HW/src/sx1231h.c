/*
 * sx1231h.c
 *  The RFM69HW uses a Semtech SX1231H module but handles all of the RF board layout
 *
 *  This driver has been ported from the API Source Code provided by Semtech under no license.
 *  http://www.semtech.com/wireless-rf/rf-transceivers/sx1231h/
 *
 *  Created on: May 13, 2015
 *      Author: Dustin
 */
#include <string.h>
#include <stdio.h>
#include "stm32l1xx.h"
#include "sx1231h.h"
#include "spi.h"
#include "uart.h"


/*******************************************************************
** Global variables                                               **
*******************************************************************/
static   uint8_t RFState = RF_STOP;     // RF state machine
static   uint8_t *pRFFrame;             // Pointer to the RF frame
static   uint8_t RFFramePos;            // RF payload current position
static   uint8_t RFFrameSize;           // RF payload size
static   uint16_t ByteCounter = 0;       // RF payload byte counter
static   uint8_t PreMode = RF_STANDBY;  // Previous chip operating mode
static   uint8_t SyncSize = 8;          // Size of sync word
static   uint8_t SyncValue[8];          // Value of sync word
static   uint32_t RFFrameTimeOut = RF_FRAME_TIMEOUT(1200); // Reception counter value (full frame timeout generation)


uint16_t RegistersCfg[] = { // SX1231 configuration registers values
		DEF_FIFO, // Left for convenience, not to be changed
		DEF_OPMODE | RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY,
		DEF_DATAMODUL | RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00,
		DEF_BITRATEMSB | RF_BITRATEMSB_4800,
		DEF_BITRATELSB | RF_BITRATELSB_4800,
		DEF_FDEVMSB | RF_FDEVMSB_5000,
		DEF_FDEVLSB | RF_FDEVLSB_5000,
		DEF_FRFMSB | RF_FRFMSB_865,
		DEF_FRFMID | RF_FRFMID_865,
		DEF_FRFLSB | RF_FRFLSB_865,
		DEF_OSC1,
		DEF_OSC2,
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
		DEF_AGCTHRESH2 | RF_AGCTHRESH2_STEP2_3 | RF_AGCTHRESH2_STEP3_11,
		DEF_AGCTHRESH3 | RF_AGCTHRESH3_STEP4_9 | RF_AGCTHRESH3_STEP5_11,
		DEF_LNA | RF_LNA_ZIN_200 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO,
		DEF_RXBW | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5,
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

		DEF_DIOMAPPING1 | RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_00,
		DEF_DIOMAPPING2 | RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_01 | RF_DIOMAPPING2_CLKOUT_OFF,
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
		DEF_PACKETCONFIG2 | RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF,
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

void sx1231h_setRFMode(uint8_t mode){
	if (mode != PreMode){
		if (mode == RF_TRANSMITTER){
			spi_write_register(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_TRANSMITTER);
			while ((spi_read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
			PreMode = RF_TRANSMITTER;
		}else if (mode == RF_RECEIVER)
		{
			spi_write_register(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_RECEIVER);
			while ((spi_read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
			PreMode = RF_RECEIVER;
		}
		else if (mode == RF_SYNTHESIZER)
		{
			spi_write_register(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SYNTHESIZER);
			while ((spi_read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
			PreMode = RF_SYNTHESIZER;
		}
		else if (mode == RF_STANDBY)
		{
			spi_write_register(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_STANDBY);
			while ((spi_read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
			PreMode = RF_STANDBY;
		}
		else
		{	// mode == RF_SLEEP
			spi_write_register(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SLEEP);
			while ((spi_read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
			PreMode = RF_SLEEP;
		}
	}
}

void sx1231h_init(void){
	uint16_t i;

	GPIOB->BSRRH |= GPIO_Pin_12;
	i = spi_read_register(REG_VERSION);
	GPIOB->BSRRL |= GPIO_Pin_12;
	if ((i== 0x24)||(i==0x23)){
		uart_OutString("SPI SX1231 Success\r\n");
	}else{
		uart_OutString("SPI SX1231 Failure\r\n");
	}
}

uint8_t sx1231h_present(void){
	uint16_t i;

	GPIOB->BSRRH |= GPIO_Pin_12;
	i = spi_read_register(REG_VERSION);
	GPIOB->BSRRL |= GPIO_Pin_12;
	if (i== 0x24)
		return 1;
	return 0;
}



