/*
 * rtc.c
 *
 *  Created on: Mar 24, 2015
 *      Author: Dustin
 */

#include "stm32l1xx.h"
#include "uart.h"
#include "rtc.h"

RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

void RTC_Config(void)
{
	RTC_InitTypeDef RTC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC Domain */
	PWR_RTCAccessCmd(ENABLE);

	/* Clear WakeUp flag */
	PWR_ClearFlag(PWR_FLAG_WU);

	/* RTC Configuration ******************************************************/
	/* Reset RTC Domain */
	RCC_RTCResetCmd(ENABLE);
	RCC_RTCResetCmd(DISABLE);

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Calendar Configuration */
	RTC_InitStructure.RTC_AsynchPrediv = 0x1F;//0x7F;
	RTC_InitStructure.RTC_SynchPrediv	=  0x03FF;//0x120; /* (37KHz / 128) - 1 = 0x120*/
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);

	rtc_setDate(14,RTC_Month_April, 10);
	rtc_setTime(1,0,0);
	rtc_setAlarm(1,0,4,0x31,RTC_AlarmMask_All);
}

void rtc_setDate(uint8_t year, uint8_t month, uint8_t day)
{
	/* Set the date: Wednesday February 5th 2014 */
	RTC_DateStructInit(&RTC_DateStructure);
	RTC_DateStructure.RTC_Year = year;
	RTC_DateStructure.RTC_Month = month;
	RTC_DateStructure.RTC_Date = day;
	//RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Tuesday;
	RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
}

void rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	/* Set the time to 01h 00mn 00s AM */
	RTC_TimeStructInit(&RTC_TimeStructure);
	RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours   = hours;
	RTC_TimeStructure.RTC_Minutes = minutes;
	RTC_TimeStructure.RTC_Seconds = seconds;
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
}

void rtc_setAlarm(uint8_t hour, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint32_t mask)
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Set the alarm 01h:00min:04s */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0x01;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0x00;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x04;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	/* RTC_AlarmMask_All - Alarm mask hour, min and second:default Alarm generation each 1s. THis is configured to trigger the 4th second of each minute*/
	//RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_Minutes | RTC_AlarmMask_Hours | RTC_AlarmMask_DateWeekDay;  //alarm the 4th second of each minute.
	RTC_AlarmStructure.RTC_AlarmMask = mask;  //alarm each second.

	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

	/* Enable RTC Alarm A Interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm */
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	RTC_ClearFlag(RTC_FLAG_ALRAF);

	/* RTC Alarm A Interrupt Configuration */
	/* EXTI configuration */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Alarm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void RTC_Alarm_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		uart_OutString("Alarm occurred!");
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		EXTI_ClearITPendingBit(EXTI_Line17);
		/*can disable alarm once it occurs */
		//RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	}
}
