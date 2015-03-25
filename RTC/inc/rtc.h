/*
 * rtc.h
 *
 *  Created on: Mar 24, 2015
 *      Author: Dustin
 */

#ifndef RTC_H_
#define RTC_H_

void RTC_Config(void);
void rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void rtc_setDate(uint8_t year, uint8_t month, uint8_t day);
void rtc_setAlarm(uint8_t hour, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint32_t mask);
#endif /* RTC_H_ */
