/*
 * adc.c
 *
 *  Created on: Mar 22, 2015
 *      Author: Dustin
 */
#include "stdio.h"
#include "stm32l1xx.h"
#include "stm32l1xx_adc.h"
#include "adc.h"
#include "stdbool.h"

#define VDD_CORRECTION 1  /* definition for correction of VDD if <> 3V */

DMA_InitTypeDef DMA_InitStructure;
uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];
uint32_t ADC_Result, INTemperature, refAVG, tempAVG, vdd_ref, Address = 0;
float temperature_C;
TSCALIB_TypeDef calibdata;    /* field storing temp sensor calibration data */
volatile bool flag_ADCDMA_TransferComplete;

uint16_t adc_getTemp(void){
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1,
			ADC_SampleTime_384Cycles);

	/* Start ADC conversion */
	ADC_SoftwareStartConv(ADC1);

	/* wait for end of conversion */
	while((ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET));

	return ADC_GetConversionValue(ADC1);
}

uint16_t adc_getVolt(void){

	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1,
			ADC_SampleTime_384Cycles);

	/* Start ADC conversion */
	ADC_SoftwareStartConv(ADC1);

	/* wait for end of conversion */
	while((ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET));

	return ADC_GetConversionValue(ADC1);
}


void adc_insertionSort(uint16_t *numbers, uint32_t array_size)
{

	uint32_t i, j;
	uint32_t index;

	for (i=1; i < array_size; i++) {
		index = numbers[i];
		j = i;
		while ((j > 0) && (numbers[j-1] > index)) {
			numbers[j] = numbers[j-1];
			j = j - 1;
		}
		numbers[j] = index;
	}
}

uint32_t adc_interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
	uint32_t sum=0;
	uint32_t  index, maxindex;
	/* discard  the lowest and the highest data samples */
	maxindex = 3 * numOfSamples / 4;
	for (index = (numOfSamples / 4); index < maxindex; index++){
		sum += array[index];
	}
	/* return the mean value of the remaining samples value*/
	return ( sum / (numOfSamples / 2) );
}



void adc_setADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = true;
}

void adc_clearADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = false;
}

/**
  * @brief  This function handles DMA Transfer Complete interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler    (void)
{
  DMA_ClearFlag(DMA1_IT_TC1);
  adc_setADCDMA_TransferComplete();  /* set flag_ADCDMA_TransferComplete global flag */
}

void adc_acquireTemperatureData(void)
{
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);

  /* re-initialize DMA -- is it needed ?*/
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Enable DMA channel 1 Transmit complete interrupt*/
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Disable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, DISABLE);

   /* Enable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, ENABLE);

  /* Clear global flag for DMA transfert complete */
  adc_clearADCDMA_TransferComplete();

  /* Start ADC conversion */
  ADC_SoftwareStartConv(ADC1);
}

void adc_processTempData(void)
{
	uint32_t index,dataSum;
	dataSum = 0;

	/* sort received data in */
	adc_insertionSort(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);

	/* Calculate the Interquartile mean */
	tempAVG = adc_interquartileMean(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);

	/* Sum up all mesured data for reference voltage average calculation */
	for (index = MAX_TEMP_CHNL; index < ADC_CONV_BUFF_SIZE; index++){
		dataSum += ADC_ConvertedValueBuff[index];
	}
	/* Devide sum up result by 4 for the temperature average calculation*/
	refAVG = dataSum / 4 ;

#if VDD_CORRECTION == 1
	/* estimation of VDD=VDDA=Vref+ from ADC measurement of Vrefint reference voltage in mV */
	vdd_ref = calibdata.VREF * 3300 / refAVG;

	/* correction factor if VDD <> 3V */
	tempAVG = tempAVG * vdd_ref / 3300;

#endif

	/* Calculate temperature in °C from Interquartile mean */
	temperature_C = ( (float) tempAVG - (float) calibdata.TS_CAL_1 ) ;
	temperature_C = temperature_C * (float)(HOT_CAL_TEMP - COLD_CAL_TEMP);
	temperature_C = temperature_C /
			(float)(calibdata.TS_CAL_2 - calibdata.TS_CAL_1);
	temperature_C = temperature_C + COLD_CAL_TEMP;
}

float adc_getCalTemp(void)
{
	adc_processTempData();
	return temperature_C;
}

void adc_getFactoryTSCalibData(TSCALIB_TypeDef* data)
{
	uint32_t deviceID;

	/*DEV_ID(11:0): Device identifier
	This field indicates the device ID.
	0x416: Cat.1 device
	0x429: Cat.2 device
	0x427: Cat.3 device
	0x436: Cat.4 device or Cat.3 device(1)
	0x437: Cat.5 device or Cat.6 device
	 */
	deviceID = DBGMCU_GetDEVID();

	if (deviceID == 0x437) *data = *FACTORY_TSCALIB_MD_DATA;
	else while(1); // add error handler - device cannot be identified calibration data not loaded!
}

ErrorStatus  adc_testFactoryCalibData(void)
{
	int32_t testdiff;
	ErrorStatus retval = ERROR;
	TSCALIB_TypeDef datatotest;

	adc_getFactoryTSCalibData (&datatotest);

	testdiff = datatotest.TS_CAL_2 - datatotest.TS_CAL_1;

	if ( testdiff > TEST_CALIB_DIFF )    retval = SUCCESS;

	return retval;
}



int adc_init(void)
{
	uint32_t ch_index;
	__IO uint16_t T_StartupTimeDelay;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Test user or factory temperature sensor calibration value */
	if ( adc_testFactoryCalibData() == SUCCESS ) {
		//uart_OutString("Factory Calib good\r\n");
		adc_getFactoryTSCalibData(&calibdata);
	}

	/* Enable ADC clock & SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Enable the internal connection of Temperature sensor and with the ADC channels*/
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Wait until ADC + Temp sensor start */
	T_StartupTimeDelay = 1024;
	while (T_StartupTimeDelay--);

	/* Setup ADC common init struct */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* Initialise the ADC1 by using its init structure */
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	          // Set conversion resolution to 12bit
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          // Enable Scan mode (single conversion for each channel of the group)
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  // Disable Continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
	ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular Temperature sensor channel16 and internal reference channel17 configuration */

	for (ch_index = 1; ch_index <= MAX_TEMP_CHNL; ch_index++){
		ADC_RegularChannelConfig(ADC1, ADC_Channel_16, ch_index,
				ADC_SampleTime_384Cycles);
	}

	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 17, ADC_SampleTime_384Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 18, ADC_SampleTime_384Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 19, ADC_SampleTime_384Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 20, ADC_SampleTime_384Cycles);
}

void adc_configureDMA(void)
{
	/* Declare NVIC init Structure */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* De-initialise  DMA */
	DMA_DeInit(DMA1_Channel1);

	/* DMA1 channel1 configuration */
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     // Set DMA channel Peripheral base address to ADC Data register
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  // Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         // Set DMA channel direction to peripheral to memory
	DMA_InitStructure.DMA_BufferSize = ADC_CONV_BUFF_SIZE;                     // Set DMA channel buffersize to peripheral to ADC_CONV_BUFF_SIZE
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     // Disable DMA channel Peripheral address auto increment
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // Enable Memeory increment (To be verified ....)
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// set Peripheral data size to 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     // set Memeory data size to 8bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                              // Set DMA in normal mode
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     // Set DMA channel priority to High
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                               // Disable memory to memory option
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								 // Use Init structure to initialise channel1 (channel linked to ADC)

	/* Enable Transmit Complete Interrup for DMA channel 1 */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	/* Setup NVIC for DMA channel 1 interrupt request */
	NVIC_InitStructure.NVIC_IRQChannel =   DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

uint16_t adc_read(ADC_TypeDef* ADCx, uint8_t channel, uint8_t ADC_SampleTime) {
	/* Configure Channel */
	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime);

	/* check if conversion was started, if not start */
	ADC_SoftwareStartConv(ADCx);

	/* wait for end of conversion */
	while((ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET));

	return ADC_GetConversionValue(ADCx);
}
