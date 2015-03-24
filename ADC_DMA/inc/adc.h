/*
 * adc.h
 *
 *  Created on: Mar 22, 2015
 *      Author: Dustin
 */

#ifndef ADC_H_
#define ADC_H_

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_1; // low temperature calibration data
    uint16_t reserved;
    uint16_t TS_CAL_2; // high temperature calibration data
} TSCALIB_TypeDef;

/*STM32072 - AN3964 Temperature Sensor http://www.st.com/web/en/catalog/tools/PF257908# */
#define FACTORY_TSCALIB_MD_BASE         ((uint32_t)0x1FF800F8)    /*!< Calibration Data Bytes base address for medium density devices*/
#define FACTORY_TSCALIB_MD_DATA         ((TSCALIB_TypeDef *) FACTORY_TSCALIB_MD_BASE)
#define TEST_CALIB_DIFF           (int32_t) 50  /* difference of hot-cold calib
                                               data to be considered as valid */
#define HOT_CAL_TEMP 		110
#define COLD_CAL_TEMP  	30
#define ADC_CONV_BUFF_SIZE 20
#define MAX_TEMP_CHNL 16

#define ADC_CONV_BUFF_SIZE 20

float adc_getCalTemp(void);



#endif /* ADC_H_ */
