/*
 * i2c.h
 *
 *  Created on: Jun 7, 2015
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
 *
 */

#ifndef I2C_H_
#define I2C_H_

#define ACK	1
#define NACK 2

#define DS3231_TIMEOUT			1000 //number of iterations to attempt
#define DS3231_SECONDS_REG		0x00 //Seconds register
#define DS3231_MINUTES_REG		0x01
#define DS3231_HOUR_REG			0x02 //Hour, AM/PM, 12/24
#define DS3231_DAY_REG			0x03
#define DS3231_DATE_REG			0x04
#define DS3231_MONTH_REG		0x05
#define DS3231_YEAR_REG			0x06

#define DS3231_ALARM_DATE  			0 //alarm on date
#define DS3231_ALARM_DAY			1 //alarm on day match

#define DS3231_ALARM_1_SECONDS_REG	0x07 // A1M1
#define DS3231_ALARM_1_MINUTES_REG	0x08 // A1M2
#define DS3231_ALARM_1_HOUR_REG		0x09 // A1M3
#define DS3231_ALARM_1_DAY_REG		0x0A // Day/Date, 10date, Dy/DT, AIM4

#define DS3231_ALARM_2_MINUTES_REG	0x0B //Min, A2M2
#define DS3231_ALARM_2_HOUR_REG		0x0C //A2M3
#define DS3231_ALARM_2_DAY_REG		0x0D //Day/Date, 10date, Dy/dt, A2M4

#define DS3231_CONTROL_REG		0x0E //Control Register
#define DS3231_CONTROL_EOSC		0x80 //Enable Oscillator
#define DS3231_CONTROL_BBSWE	0x40 //Battery Backed Square Wave Enable
#define DS3231_CONTROL_CONV		0x20 //Convert Temperature
#define DS3231_CONTROL_RS2		0x10 //Rate Select 2
#define DS3231_CONTROL_RS1		0x08 //Rate Select 1 for Square Wave Output
#define DS3231_CONTROL_INTCN	0x04 //Interrupt Control
#define DS3231_CONTROL_A2IE		0x02 //Alarm 2 Interrupt Enable
#define DS3231_CONTROL_A1IE		0x01 //Alarm 1 Interrupt Enable
#define DS3231_I2C_ADDR 0x68

#define DS3231_CONTROL_STATUS_REG				0x0F
#define DS3231_CONTROL_STATUS_ALARM_1_FLAG 		0x01
#define DS3231_CONTROL_STATUS_ALARM_2_FLAG 		0x02

#define DS3231_AGING_OFFSET_REG		0x10
#define DS3231_MSB_TEMP_REG			0x11
#define DS3231_LSB_TEMP_REG 		0x12

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t am_pm;
	uint8_t is_ampm_time;
}DS3231_time_t;

typedef struct {
	uint8_t day; //1-7 day of the week
	uint8_t date; //1-31
	uint8_t month; //01-12
	uint8_t year; //00-99
}DS3231_date_t;

typedef struct {
	uint8_t seconds;  //Not valid for alarm 2
	uint8_t minutes;
	uint8_t hours;
	uint8_t am_pm;
	uint8_t is_ampm_time;
	uint8_t day;
	uint8_t date;
	uint8_t AMx[4];
	uint8_t day_or_date;  //AMx all 0 for this option to be considered
}DS3231_alarm_t;



void i2c_init(void);
uint8_t i2c_ds3231_init(I2C_TypeDef* I2Cx);
uint8_t i2c_ds3231_write_reg(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value);
uint8_t i2c_ds3231_read_register(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * value);
uint8_t i2c_ds3231_read_registers(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * data, uint8_t count);
uint8_t i2c_ds3231_set_time_date(I2C_TypeDef* I2Cx, DS3231_time_t time, DS3231_date_t date);
uint8_t i2c_ds3231_read_time_date(I2C_TypeDef* I2Cx, DS3231_time_t * time, DS3231_date_t * date);
uint8_t i2c_ds3231_read_temperature(I2C_TypeDef* I2Cx, float * temperature);
uint8_t i2c_ds3231_set_alarm_1(I2C_TypeDef* I2Cx, DS3231_alarm_t alarm);
uint8_t i2c_ds3231_read_alarm_1(I2C_TypeDef* I2Cx, DS3231_alarm_t * alarm);
uint8_t i2c_ds3231_set_alarm_2(I2C_TypeDef* I2Cx, DS3231_alarm_t alarm);
uint8_t i2c_ds3231_read_alarm_2(I2C_TypeDef* I2Cx, DS3231_alarm_t * alarm);
uint8_t i2c_ds3231_check_alarm_1(I2C_TypeDef* I2Cx);
uint8_t i2c_ds3231_check_alarm_2(I2C_TypeDef* I2Cx);
uint8_t i2c_ds3231_test_rtc(I2C_TypeDef* I2Cx);

#endif /* I2C_H_ */
