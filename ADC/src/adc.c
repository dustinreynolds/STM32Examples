/*
 * adc.c
 *
 *  Created on: Mar 22, 2015
 *      Author: Dustin
 */
#include "stm32l1xx.h"
#include "stm32l1xx_adc.h"
#include "adc.h"

#define VDD_CORRECTION 1  /* definition for correction of VDD if <> 3V */

DMA_InitTypeDef DMA_InitStructure;
uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];
uint32_t ADC_Result, INTemperature, refAVG, tempAVG, vdd_ref, Address = 0;
float temperature_C;
TSCALIB_TypeDef calibdata;    /* field storing temp sensor calibration data */

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

void processTempData(void)
{
	uint32_t index,dataSum;
	dataSum = 0;
	uint16_t temp;

	temp = adc_getTemp();
	refAVG = adc_getVolt();

#if VDD_CORRECTION == 1
	/* estimation of VDD=VDDA=Vref+ from ADC measurement of Vrefint reference voltage in mV */
	vdd_ref = calibdata.VREF * 3300 / refAVG;

	/* correction factor if VDD <> 3V */
	temp = temp * vdd_ref / 3300;

#endif

	/* Calculate temperature in °C from Interquartile mean */
	temperature_C = ( (float) temp - (float) calibdata.TS_CAL_1 ) ;
	temperature_C = temperature_C * (float)(HOT_CAL_TEMP - COLD_CAL_TEMP);
	temperature_C = temperature_C /
			(float)(calibdata.TS_CAL_2 - calibdata.TS_CAL_1);
	temperature_C = temperature_C + COLD_CAL_TEMP;
}

float adc_getCalTemp(void)
{
	processTempData();
	return temperature_C;
}

void getFactoryTSCalibData(TSCALIB_TypeDef* data)
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

ErrorStatus  testFactoryCalibData(void)
{
	int32_t testdiff;
	ErrorStatus retval = ERROR;
	TSCALIB_TypeDef datatotest;

	getFactoryTSCalibData (&datatotest);

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

	/* Set internal voltage regulator to 1.8V */
	PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

	/* Wait Until the Voltage Regulator is ready */
	while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;

	/* Test user or factory temperature sensor calibration value */
	if ( testFactoryCalibData() == SUCCESS ) {
		//uart_OutString("Factory Calib good\r\n");
		getFactoryTSCalibData(&calibdata);
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
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	                          // Enable Scan mode (single conversion for each channel of the group)
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  // Disable Continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
	ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until the ADC1 is ready */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);
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
