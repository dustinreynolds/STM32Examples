/*
 * i2c.c
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
 */

#include "stm32l1xx_i2c.h"
#include "string.h"
#include "i2c.h"

uint8_t _i2c_start_address(I2C_TypeDef* I2Cx, uint8_t address, uint8_t I2C_Direction);
uint8_t _i2c_stop(I2C_TypeDef* I2Cx);
uint8_t _i2c_write(I2C_TypeDef* I2Cx, uint8_t byte);
uint8_t _i2c_read(I2C_TypeDef* I2Cx, uint8_t ack, uint8_t * byte);

void i2c_init(void){
	I2C_InitTypeDef I2C_InitStructure;

	I2C_StructInit(&I2C_InitStructure);

	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2C2, &I2C_InitStructure);

	I2C_Cmd(I2C2, ENABLE);
}

void i2c_ds3231_reset(I2C_TypeDef* I2Cx){
	uint32_t count = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Toggle SCL until SDA is in a high state
	while(count < 1000){
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1){
			break;
		}
		GPIOB->ODR ^= GPIO_Pin_10; //toggle SCL
		delayms(1);
	}

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
}

uint8_t i2c_ds3231_init(I2C_TypeDef* I2Cx){
	uint8_t control_reg = 0x00;
	uint8_t result;

	control_reg = DS3231_CONTROL_RS2 | DS3231_CONTROL_RS1 | DS3231_CONTROL_INTCN;
	result = i2c_ds3231_write_reg(I2Cx, DS3231_CONTROL_REG, control_reg);
	return result;
}
uint8_t i2c_ds3231_set_address(I2C_TypeDef* I2Cx, uint8_t address){
	uint8_t result;
	result = _i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Transmitter);
	if (result) return 1;
	result = _i2c_write(I2Cx, address);
	if (result) return 1;
	result = _i2c_stop(I2Cx);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_write_reg(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value){
	uint8_t result;
	result = _i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Transmitter);
	if (result) return 1;
	result = _i2c_write(I2Cx, address);
	if (result) return 1;
	result = _i2c_write(I2Cx, value);
	if (result) return 1;
	result = _i2c_stop(I2Cx);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_write_registers(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * data, uint8_t count){
	uint8_t result;
	result = _i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Transmitter);
	if (result) return 1;
	result = _i2c_write(I2Cx, address);
	if (result) return 1;
	while (count-- > 0){
		result = _i2c_write(I2Cx, *data++);
		if (result) return 1;
	}
	result = _i2c_stop(I2Cx);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_read_register(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * value){
	uint8_t result;
	result = i2c_ds3231_set_address(I2Cx, address);
	if (result) return  1;

	result = _i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Receiver);
	if (result) return  1;
	result = _i2c_read(I2Cx, NACK, value);
	if (result) return  1;
	return 0;
}

uint8_t i2c_ds3231_read_registers(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * data, uint8_t count){
	uint8_t result;
	result = i2c_ds3231_set_address(I2Cx, address);
	if (result) return 1;

	result = _i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Receiver);
	if (result) return 1;
	while (--count > 0){
		result = _i2c_read(I2Cx, ACK, data++);
		if (result) return 1;
	}
	result = _i2c_read(I2Cx, NACK, data);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_set_time_date(I2C_TypeDef* I2Cx, DS3231_time_t time, DS3231_date_t date){
	uint8_t reg;
	uint8_t result;

	//Write to registers
	reg = ((time.seconds / 10) << 4 ) | time.seconds % 10;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_SECONDS_REG, reg);
	if (result) return 1;

	reg = ((time.minutes / 10) << 4 ) | time.minutes % 10;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_MINUTES_REG, reg);
	if (result) return 1;

	if(time.is_ampm_time){
		reg = (time.is_ampm_time << 6) | (time.am_pm << 5) | ((time.hours / 10) << 4 ) | time.hours % 10;
	}else{
		reg = (time.is_ampm_time << 6) | ((time.hours / 10) << 4 ) | time.hours % 10;
	}
	result = i2c_ds3231_write_reg(I2Cx,DS3231_HOUR_REG, reg);
	if (result) return 1;

	reg = date.day;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_DAY_REG, reg);
	if (result) return 1;

	reg = ((date.date / 10) << 4 ) | date.date % 10;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_DATE_REG, reg);
	if (result) return 1;

	reg = ((date.month / 10) << 4) | date.month % 10;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_MONTH_REG, reg);
	if (result) return 1;

	reg = ((date.year / 10) << 4) | date.year % 10;
	result = i2c_ds3231_write_reg(I2Cx,DS3231_YEAR_REG, reg);
	if (result) return 1;
}

uint8_t i2c_ds3231_read_time_date(I2C_TypeDef* I2Cx, DS3231_time_t * time, DS3231_date_t * date){
	uint8_t temp[7];
	uint8_t result;
	result = i2c_ds3231_read_registers(I2Cx, 0x00, temp, 7);
	if (result) return 1;

	time->seconds = ((temp[0] & 0x70) >> 4) * 10 + (temp[0] & 0x0F);
	time->minutes = ((temp[1] & 0x70) >> 4) * 10 + (temp[1] & 0x0F);
	if ((temp[2] & 0x40) > 0){
		time->is_ampm_time = 1;
		time->am_pm = (temp[2] & 0x20) >> 5;
		time->hours = ((temp[2] & 0x10) >> 4) * 10 + (temp[2] & 0x0F);
	}else{
		time->is_ampm_time = 0;
		time->am_pm = 0;
		time->hours = ((temp[2] & 0x30) >> 4) * 10 + (temp[2] & 0x0F);
	}

	date->day = temp[3];
	date->date = ((temp[4] & 0x30 ) >> 4) * 10 + (temp[4] & 0x0F);

	date->month = ((temp[5] & 0x10 ) >> 4 ) * 10 + (temp[5] & 0x0F);
	date->year = ((temp[6] & 0xFF) >> 4 ) * 10 + (temp[6] & 0x0F);
	return 0;
}

uint8_t i2c_ds3231_read_temperature(I2C_TypeDef* I2Cx, float * temperature){
	uint8_t temp[2];
	uint8_t result;
	result = i2c_ds3231_read_registers(I2Cx, DS3231_MSB_TEMP_REG, temp, 2);
	if (result) return 1;
	//http://ideone.com/seM4DH
	if ((temp[0] & 0x80) > 0){
		temperature[0] = (char)temp[0] + ((float)(temp[1] >> 6))/4.0;
	}else{
		temperature[0] = temp[0] + ((float)(temp[1] >> 6))/4.0;
	}
	return 0;
}

uint8_t i2c_ds3231_set_alarm_1(I2C_TypeDef* I2Cx, DS3231_alarm_t alarm){
	//given alarm settings, set alarm accordingly
	uint8_t temp[4];
	uint8_t result;

	temp[0] = (alarm.AMx[0] << 7) | ((alarm.seconds / 10) << 4 ) | alarm.seconds % 10;
	temp[1] = (alarm.AMx[1] << 7) | ((alarm.minutes / 10) << 4 ) | alarm.minutes % 10;
	if(alarm.is_ampm_time){
		temp[2] = (alarm.AMx[2] << 7) | (alarm.is_ampm_time << 6) | (alarm.am_pm << 5) | ((alarm.hours / 10) << 4 ) | alarm.hours % 10;
	}else{
		temp[2] = (alarm.AMx[2] << 7) | (alarm.is_ampm_time << 6) | ((alarm.hours / 10) << 4 ) | alarm.hours % 10;
	}

	if(alarm.day_or_date){
		temp[3] = (alarm.AMx[3] << 7) | (alarm.day_or_date << 6) | alarm.day;
	}else{
		temp[3] = (alarm.AMx[3] << 7) | (alarm.day_or_date << 6) | ((alarm.date / 10) << 4 ) | alarm.date % 10;
	}

	result = i2c_ds3231_write_registers(I2Cx, DS3231_ALARM_1_SECONDS_REG, temp, 4);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_read_alarm_1(I2C_TypeDef* I2Cx, DS3231_alarm_t * alarm){
	uint8_t temp[4];
	uint8_t result;
	result = i2c_ds3231_read_registers(I2Cx, DS3231_ALARM_1_SECONDS_REG, temp, 4);
	if (result) return 1;

	alarm->seconds = ((temp[0] & 0x70) >> 4) * 10 + (temp[0] & 0x0F);
	alarm->AMx[0] = (temp[0] & 0x80) >> 7;
	alarm->minutes = ((temp[1] & 0x70) >> 4) * 10 + (temp[1] & 0x0F);
	alarm->AMx[1] = (temp[1] & 0x80) >> 7;

	if ((temp[2] & 0x40) > 0){
		alarm->is_ampm_time = 1;
		alarm->am_pm = (temp[2] & 0x20) >> 5;
		alarm->hours = ((temp[2] & 0x10) >> 4) * 10 + (temp[2] & 0x0F);
	}else{
		alarm->is_ampm_time = 0;
		alarm->am_pm = 0;
		alarm->hours = ((temp[2] & 0x30) >> 4) * 10 + (temp[2] & 0x0F);
	}
	alarm->AMx[2] = (temp[2] & 0x80) >> 7;

	if ((temp[3] & 0x40) > 0){
		alarm->day = temp[3] & 0x0F;
		alarm->date = 0;
	}else{

		alarm->date = ((temp[3] & 0x30 ) >> 4) * 10 + (temp[3] & 0x0F);
		alarm->day = 0;
	}
	alarm->day_or_date = (temp[3] & 0x40) >> 6;
	alarm->AMx[3] = (temp[3] & 0x80) >> 7;
	return 0;
}

uint8_t i2c_ds3231_set_alarm_2(I2C_TypeDef* I2Cx, DS3231_alarm_t alarm){
	//given alarm settings, set alarm accordingly
	uint8_t temp[3];
	uint8_t result;

	temp[0] = (alarm.AMx[1] << 7) | ((alarm.minutes / 10) << 4 ) | alarm.minutes % 10;
	if(alarm.is_ampm_time){
		temp[1] = (alarm.AMx[2] << 7) | (alarm.is_ampm_time << 6) | (alarm.am_pm << 5) | ((alarm.hours / 10) << 4 ) | alarm.hours % 10;
	}else{
		temp[1] = (alarm.AMx[2] << 7) | (alarm.is_ampm_time << 6) | ((alarm.hours / 10) << 4 ) | alarm.hours % 10;
	}

	if(alarm.day_or_date){
		temp[2] = (alarm.AMx[3] << 7) | (alarm.day_or_date << 6) | alarm.day;
	}else{
		temp[2] = (alarm.AMx[3] << 7) | (alarm.day_or_date << 6) | ((alarm.date / 10) << 4 ) | alarm.date % 10;
	}

	result = i2c_ds3231_write_registers(I2Cx, DS3231_ALARM_2_MINUTES_REG, temp, 3);
	if (result) return 1;
	return 0;
}

uint8_t i2c_ds3231_read_alarm_2(I2C_TypeDef* I2Cx, DS3231_alarm_t * alarm){
	uint8_t temp[3];
	uint8_t result;
	result = i2c_ds3231_read_registers(I2Cx, DS3231_ALARM_2_MINUTES_REG, temp, 3);
	if (result) return 1;

	alarm->minutes = ((temp[0] & 0x70) >> 4) * 10 + (temp[0] & 0x0F);
	alarm->AMx[1] = (temp[0] & 0x80) >> 7;

	if ((temp[1] & 0x40) > 0){
		alarm->is_ampm_time = 1;
		alarm->am_pm = (temp[1] & 0x20) >> 5;
		alarm->hours = ((temp[1] & 0x10) >> 4) * 10 + (temp[1] & 0x0F);
	}else{
		alarm->is_ampm_time = 0;
		alarm->am_pm = 0;
		alarm->hours = ((temp[1] & 0x30) >> 4) * 10 + (temp[1] & 0x0F);
	}
	alarm->AMx[2] = (temp[1] & 0x80) >> 7;

	if ((temp[2] & 0x40) > 0){
		alarm->day = temp[2] & 0x0F;
		alarm->date = 0;
	}else{

		alarm->date = ((temp[2] & 0x30 ) >> 4) * 10 + (temp[2] & 0x0F);
		alarm->day = 0;
	}
	alarm->day_or_date = (temp[2] & 0x40) >> 6;
	alarm->AMx[3] = (temp[2] & 0x80) >> 7;
	return 0;
}

uint8_t i2c_ds3231_check_alarm_1(I2C_TypeDef* I2Cx){
	uint8_t temp;
	i2c_ds3231_read_register(I2Cx, DS3231_CONTROL_STATUS_REG, &temp);

	if ((temp & DS3231_CONTROL_STATUS_ALARM_1_FLAG) > 0){
		//clear flag
		temp = temp & ~DS3231_CONTROL_STATUS_ALARM_1_FLAG; //check this
		i2c_ds3231_write_reg(I2Cx, DS3231_CONTROL_STATUS_REG,temp);

		return 1;
	}
	return 0;
}
uint8_t i2c_ds3231_check_alarm_2(I2C_TypeDef* I2Cx){
	uint8_t temp;
	i2c_ds3231_read_register(I2Cx, DS3231_CONTROL_STATUS_REG, &temp);

	if ((temp & DS3231_CONTROL_STATUS_ALARM_2_FLAG) > 0){
		//clear flag
		temp = temp & ~DS3231_CONTROL_STATUS_ALARM_2_FLAG; //check this
		i2c_ds3231_write_reg(I2Cx, DS3231_CONTROL_STATUS_REG,temp);

		return 1;
	}
	return 0;
}

unsigned char i2c_ds3231_logic_test(DS3231_time_t time, DS3231_date_t date){
	//Function to quickly test if we can get data into correct form and out of form
	DS3231_time_t time2;
	DS3231_date_t date2;
	unsigned char temp[8], reg;
	unsigned char result = 0;

	//Write to registers
	reg = ((time.seconds / 10) << 4 ) | time.seconds % 10;
	temp[0] = reg;

	reg = ((time.minutes / 10) << 4 ) | time.minutes % 10;
	temp[1] = reg;

	if(time.is_ampm_time){
		reg = (time.is_ampm_time << 6) | (time.am_pm << 5) | ((time.hours / 10) << 4 ) | time.hours % 10;
	}else{
		reg = (time.is_ampm_time << 6) | ((time.hours / 10) << 4 ) | time.hours % 10;
	}
	temp[2] = reg;

	reg = date.day;
	temp[3] = reg;

	reg = ((date.date / 10) << 4 ) | date.date % 10;
	temp[4] = reg;

	reg = ((date.month / 10) << 4) | date.month % 10;
	temp[5] = reg;

	reg = ((date.year / 10) << 4) | date.year % 10;
	temp[6] = reg;

	//read from registers
	time2.seconds = ((temp[0] & 0x70) >> 4) * 10 + (temp[0] & 0x0F);
	time2.minutes = ((temp[1] & 0x70) >> 4) * 10 + (temp[1] & 0x0F);
	if ((temp[2] & 0x40) > 0){
		time2.is_ampm_time = 1;
		time2.am_pm = (temp[2] & 0x20) >> 5;
		time2.hours = ((temp[2] & 0x10) >> 4) * 10 + (temp[2] & 0x0F);
	}else{
		time2.is_ampm_time = 0;
		time2.am_pm = 0;
		time2.hours = ((temp[2] & 0x30) >> 4) * 10 + (temp[2] & 0x0F);
	}

	date2.day = temp[3];
	date2.date = ((temp[4] & 0x30 ) >> 4) * 10 + (temp[4] & 0x0F);

	date2.month = ((temp[5] & 0x10 ) >> 4 ) * 10 + (temp[5] & 0x0F);
	date2.year = ((temp[6] & 0xFF) >> 4 ) * 10 + (temp[6] & 0x0F);

	//Test logic
	if (time.seconds != time2.seconds){
		printf("Seconds: %02d vs %02d, temp %02x", time.seconds, time2.seconds, temp[0]);
		result++;
	}if (time.minutes != time2.minutes){
		printf("minutes: %02d vs %02d, temp %02x", time.minutes, time2.minutes,temp[1]);
		result++;
	}if (time.hours != time2.hours){
		printf("hours: %02d vs %02d, temp %02x", time.hours, time2.hours,temp[2]);
		result++;
	}if (time.am_pm != time2.am_pm){
		printf("am_pm: %02d vs %02d, temp %02x", time.am_pm, time2.am_pm, temp[2]);
		result++;
	}if (time.is_ampm_time != time2.is_ampm_time){
		printf("ampm: %02d vs %02d, temp %02x", time.is_ampm_time, time2.is_ampm_time,temp[2]);
		result++;
	}

	if (date.day != date2.day){
		printf("day: %02d vs %02d, temp %02x", date.day, date2.day,temp[3]);
		result++;
	}if (date.date != date2.date){
		printf("date: %02d vs %02d, temp %02x", date.date, date2.date,temp[4]);
		result++;
	}if (date.month != date2.month){
		printf("month: %02d vs %02d, temp %02x", date.month, date2.month,temp[5]);
		result++;
	}if (date.year != date2.year){
		printf("year: %02d vs %02d, temp %02x", date.year, date2.year,temp[6]);
		result++;
	}
	return result;
}

uint8_t i2c_ds3231_test(void){
	//test all possible combinations of date and time
	//this takes a long time even for desktop processors

	uint8_t result = 0;
	uint8_t ts,tm,th,ta,ti;  //time
	uint8_t day,da,dm,dy;  //date
	unsigned long long iteration = 0;
	DS3231_time_t time;
	DS3231_date_t date;

	for(dy=0; dy<1;dy++){
		for(dm=1; dm < 13; dm++){
			for(da=1; da<32; da++){
				for(day=1;day<8;day++){
					//am_pm
					for(ti=1,ta=0;ta<2;ta++){
						for(th=1;th < 13;th++){
							for(tm=0;tm<60;tm++){
								for(ts=0;ts<60;ts++){
									time.seconds = ts;
									time.minutes = tm;
									time.hours = th;
									time.is_ampm_time = ti;
									time.am_pm = ta;
									date.day = day;
									date.date = da;
									date.month = dm;
									date.year = dy;
									result = i2c_ds3231_logic_test(time, date);
									if (result > 0){
										result = i2c_ds3231_logic_test(time, date);
										printf("result %d y %02d m %02d date %02d day %02d",result, dy,dm,da,day);
										printf(" h %02d m %02d s %02d  is_ampm %02d am_pm %02d\r\n", th,tm,ts,ti,ta);
										return 1;
									}
									iteration++;
								}
							}
						}
					}
					//24hr
					for(ti=0,th=0;th<24;th++){
						for(tm=0;tm<60;tm++){
							for(ts=0;ts<60;ts++){
								time.seconds = ts;
								time.minutes = tm;
								time.hours = th;
								time.is_ampm_time = ti;
								time.am_pm = 0;
								date.day = day;
								date.date = da;
								date.month = dm;
								date.year = dy;
								result = i2c_ds3231_logic_test(time, date);
								if (result > 0){
									result = i2c_ds3231_logic_test(time, date);
									printf("result %d y %02d m %02d date %02d day %02d",result, dy,dm,da,day);
									printf(" h %02d m %02d s %02d  is_ampm %02d am_pm %02d\r\n", th,tm,ts,ti,ta);
									return 1;
								}
								iteration++;
							}
						}
					}
				}
			}
			printf("Iterations %d",iteration);
			printf(" y %02d m %02d date %02d day %02d",dy,dm,da,day);
			printf(" h %02d m %02d s %02d  is_ampm %02d am_pm %02d\r\n", th,tm,ts,ti,ta);
		}
	}

	return 0;
}

uint8_t i2c_ds3231_test_rtc(I2C_TypeDef* I2Cx){
	DS3231_time_t time1, time2;
	DS3231_date_t date1, date2;
	uint8_t error_count = 0;
	uint8_t result = 0;

	time1.seconds = 0;
	time1.minutes = 2;
	time1.hours = 9;
	time1.is_ampm_time = 0;
	time1.am_pm = 0;
	date1.day = 1;
	date1.date = 14;
	date1.month = 6;
	date1.year = 15;
	result = i2c_ds3231_set_time_date(I2Cx, time1, date1);
	if(result) return 1;

	delayms(2100);

	result = i2c_ds3231_read_time_date(I2Cx, &time2, &date2);
	if(result) return 1;

	if(time2.seconds != (time1.seconds + 2) ){
		error_count++;
	}if(time2.minutes != time1.minutes){
		error_count++;
	}if(time2.hours != time1.hours){
		error_count++;
	}if(time2.is_ampm_time != time1.is_ampm_time){
		error_count++;
	}if(time2.am_pm != time1.am_pm){
		error_count++;
	}if(date2.day != date1.day){
		error_count++;
	}if(date2.date != date1.date){
		error_count++;
	}if(date2.month != date1.month){
		error_count++;
	}if(date2.year != date1.year){
		error_count++;
	}

	if (error_count)	return 1;
	//Test Alarm 1 and 2
	{
		{ //Test once a second alarm 1
			DS3231_alarm_t alarm1, alarm1v;
			uint8_t alarm1_state[2];

			time1.seconds = 0;
			time1.minutes = 2;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = time1.seconds+1;
			alarm1.minutes = time1.minutes;
			alarm1.hours = time1.hours;
			alarm1.am_pm = time1.am_pm;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = 0;
			alarm1.date = date1.date;
			alarm1.AMx[0] = 1; alarm1.AMx[1] = 1;alarm1.AMx[2] = 1;alarm1.AMx[3] = 1;
			alarm1.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states
			result += i2c_ds3231_read_alarm_1(I2Cx, &alarm1v);
			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			//check that we can readback the alarm in full detail.
			if (alarm1.seconds != alarm1v.seconds){
				error_count++;
			}if (alarm1.minutes != alarm1v.minutes){
				error_count++;
			}if (alarm1.hours != alarm1v.hours){
				error_count++;
			}if (alarm1.am_pm != alarm1v.am_pm){
				error_count++;
			}if (alarm1.is_ampm_time != alarm1v.is_ampm_time){
				error_count++;
			}if (alarm1.day != alarm1v.day){
				error_count++;
			}if (alarm1.date != alarm1v.date){
				error_count++;
			}if (alarm1.AMx[0] != alarm1v.AMx[0]){
				error_count++;
			}if (alarm1.AMx[1] != alarm1v.AMx[1]){
				error_count++;
			}if (alarm1.AMx[2] != alarm1v.AMx[2]){
				error_count++;
			}if (alarm1.AMx[3] != alarm1v.AMx[3]){
				error_count++;
			}if (alarm1.day_or_date != alarm1v.day_or_date){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 once a second %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if(result) return 1;
		}

		{ //Test seconds match alarm 1
			DS3231_alarm_t alarm1;
			uint8_t alarm1_state[2];

			time1.seconds = 1;
			time1.minutes = 2;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = time1.seconds+1;
			alarm1.minutes = time1.minutes;
			alarm1.hours = time1.hours;
			alarm1.am_pm = time1.am_pm;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = date1.day;
			alarm1.date = date1.date;
			alarm1.AMx[0] = 0; alarm1.AMx[1] = 1;alarm1.AMx[2] = 1;alarm1.AMx[3] = 1;
			alarm1.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 seconds match %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test minutes and seconds match alarm 1
			DS3231_alarm_t alarm1;
			uint8_t alarm1_state[2];

			time1.seconds = 59;
			time1.minutes = 2;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = 0;
			alarm1.minutes = time1.minutes+1;
			alarm1.hours = time1.hours;
			alarm1.am_pm = time1.am_pm;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = date1.day;
			alarm1.date = date1.date;
			alarm1.AMx[0] = 0; alarm1.AMx[1] = 0;alarm1.AMx[2] = 1;alarm1.AMx[3] = 1;
			alarm1.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 min sec %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test hours, minutes and seconds match alarm 1
			DS3231_alarm_t alarm1;
			uint8_t alarm1_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = 0;
			alarm1.minutes = 0;
			alarm1.hours = time1.hours+1;
			alarm1.am_pm = time1.am_pm;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = date1.day;
			alarm1.date = date1.date;
			alarm1.AMx[0] = 0; alarm1.AMx[1] = 0;alarm1.AMx[2] = 0;alarm1.AMx[3] = 1;
			alarm1.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 hr min sec %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test date, hours, minutes and seconds match alarm 1
			DS3231_alarm_t alarm1;
			uint8_t alarm1_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 11;
			time1.is_ampm_time = 1;
			time1.am_pm = 1;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = 0;
			alarm1.minutes = 0;
			alarm1.hours = 12;
			alarm1.am_pm = 0;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = date1.day;
			alarm1.date = date1.date+1;
			alarm1.AMx[0] = 0; alarm1.AMx[1] = 0;alarm1.AMx[2] = 0;alarm1.AMx[3] = 0;
			alarm1.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 date hr min sec %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test day, hours, minutes and seconds match alarm 1
			DS3231_alarm_t alarm1;
			uint8_t alarm1_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 11;
			time1.is_ampm_time = 1;
			time1.am_pm = 1;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm1.seconds = 0;
			alarm1.minutes = 0;
			alarm1.hours = 12;
			alarm1.am_pm = 0;
			alarm1.is_ampm_time = time1.is_ampm_time;
			alarm1.day = date1.day+1;
			alarm1.date = date1.date+1;
			alarm1.AMx[0] = 0; alarm1.AMx[1] = 0;alarm1.AMx[2] = 0;alarm1.AMx[3] = 0;
			alarm1.day_or_date = DS3231_ALARM_DAY;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_1(I2Cx, alarm1);
			i2c_ds3231_check_alarm_1(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm1_state[0] = i2c_ds3231_check_alarm_1(I2Cx);
			delayms(300);
			alarm1_state[1] = i2c_ds3231_check_alarm_1(I2Cx);
			if(alarm1_state[0] != 0){
				error_count++;
			}if (alarm1_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm1 day hr min sec %d %d\r\n", alarm1_state[0],alarm1_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}


		{ //Test once a minute alarm 2
			DS3231_alarm_t alarm2, alarm2v;
			uint8_t alarm2_state[2];

			time1.seconds = 59;
			time1.minutes = 2;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm2.minutes = time1.minutes+1;
			alarm2.hours = time1.hours;
			alarm2.am_pm = time1.am_pm;
			alarm2.is_ampm_time = time1.is_ampm_time;
			alarm2.day = 0;
			alarm2.date = date1.date;
			alarm2.AMx[1] = 1;alarm2.AMx[2] = 1;alarm2.AMx[3] = 1;
			alarm2.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_2(I2Cx, alarm2);
			i2c_ds3231_check_alarm_2(I2Cx); //clear any existing alarm states
			result += i2c_ds3231_read_alarm_2(I2Cx, &alarm2v);

			//wait and check
			delayms(800);
			alarm2_state[0] = i2c_ds3231_check_alarm_2(I2Cx);
			delayms(300);
			alarm2_state[1] = i2c_ds3231_check_alarm_2(I2Cx);
			if(alarm2_state[0] != 0){
				error_count++;
			}if (alarm2_state[1] != 1){
				error_count++;
			}
			//check that we can readback the alarm in full detail.
			if (alarm2.minutes != alarm2v.minutes){
				error_count++;
			}if (alarm2.hours != alarm2v.hours){
				error_count++;
			}if (alarm2.am_pm != alarm2v.am_pm){
				error_count++;
			}if (alarm2.is_ampm_time != alarm2v.is_ampm_time){
				error_count++;
			}if (alarm2.day != alarm2v.day){
				error_count++;
			}if (alarm2.date != alarm2v.date){
				error_count++;
			}if (alarm2.AMx[1] != alarm2v.AMx[1]){
				error_count++;
			}if (alarm2.AMx[2] != alarm2v.AMx[2]){
				error_count++;
			}if (alarm2.AMx[3] != alarm2v.AMx[3]){
				error_count++;
			}if (alarm2.day_or_date != alarm2v.day_or_date){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm2 once a minute %d %d\r\n", alarm2_state[0],alarm2_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test minutes match alarm 2
			DS3231_alarm_t alarm2;
			uint8_t alarm2_state[2];

			time1.seconds = 59;
			time1.minutes = 2;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm2.minutes = time1.minutes+1;
			alarm2.hours = time1.hours;
			alarm2.am_pm = time1.am_pm;
			alarm2.is_ampm_time = time1.is_ampm_time;
			alarm2.day = date1.day;
			alarm2.date = date1.date;
			alarm2.AMx[1] = 0;alarm2.AMx[2] = 1;alarm2.AMx[3] = 1;
			alarm2.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_2(I2Cx, alarm2);
			i2c_ds3231_check_alarm_2(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm2_state[0] = i2c_ds3231_check_alarm_2(I2Cx);
			delayms(300);
			alarm2_state[1] = i2c_ds3231_check_alarm_2(I2Cx);
			if(alarm2_state[0] != 0){
				error_count++;
			}if (alarm2_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm2 min %d %d\r\n", alarm2_state[0],alarm2_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test hours and minutes match alarm 2
			DS3231_alarm_t alarm2;
			uint8_t alarm2_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 9;
			time1.is_ampm_time = 0;
			time1.am_pm = 0;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm2.minutes = 0;
			alarm2.hours = time1.hours+1;
			alarm2.am_pm = time1.am_pm;
			alarm2.is_ampm_time = time1.is_ampm_time;
			alarm2.day = date1.day;
			alarm2.date = date1.date;
			alarm2.AMx[1] = 0;alarm2.AMx[2] = 0;alarm2.AMx[3] = 1;
			alarm2.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_2(I2Cx, alarm2);
			i2c_ds3231_check_alarm_2(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm2_state[0] = i2c_ds3231_check_alarm_2(I2Cx);
			delayms(300);
			alarm2_state[1] = i2c_ds3231_check_alarm_2(I2Cx);
			if(alarm2_state[0] != 0){
				error_count++;
			}if (alarm2_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm2 hr min %d %d\r\n", alarm2_state[0],alarm2_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test date hours and minutes match alarm 2
			DS3231_alarm_t alarm2;
			uint8_t alarm2_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 11;
			time1.is_ampm_time = 1;
			time1.am_pm = 1;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm2.minutes = 0;
			alarm2.hours = 12;
			alarm2.am_pm = 0;
			alarm2.is_ampm_time = time1.is_ampm_time;
			alarm2.day = date1.day;
			alarm2.date = date1.date+1;
			alarm2.AMx[1] = 0;alarm2.AMx[2] = 0;alarm2.AMx[3] = 0;
			alarm2.day_or_date = DS3231_ALARM_DATE;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_2(I2Cx, alarm2);
			i2c_ds3231_check_alarm_2(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm2_state[0] = i2c_ds3231_check_alarm_2(I2Cx);
			delayms(300);
			alarm2_state[1] = i2c_ds3231_check_alarm_2(I2Cx);
			if(alarm2_state[0] != 0){
				error_count++;
			}if (alarm2_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm2 date hr min %d %d\r\n", alarm2_state[0],alarm2_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}

		{ //Test date hours and minutes match alarm 2
			DS3231_alarm_t alarm2;
			uint8_t alarm2_state[2];

			time1.seconds = 59;
			time1.minutes = 59;
			time1.hours = 11;
			time1.is_ampm_time = 1;
			time1.am_pm = 1;
			date1.day = 1;
			date1.date = 14;
			date1.month = 6;
			date1.year = 15;

			alarm2.minutes = 0;
			alarm2.hours = 12;
			alarm2.am_pm = 0;
			alarm2.is_ampm_time = time1.is_ampm_time;
			alarm2.day = date1.day+1;
			alarm2.date = date1.date+1;
			alarm2.AMx[1] = 0;alarm2.AMx[2] = 0;alarm2.AMx[3] = 0;
			alarm2.day_or_date = DS3231_ALARM_DAY;

			result += i2c_ds3231_set_time_date(I2Cx, time1, date1);

			result += i2c_ds3231_set_alarm_2(I2Cx, alarm2);
			i2c_ds3231_check_alarm_2(I2Cx); //clear any existing alarm states

			//wait and check
			delayms(800);
			alarm2_state[0] = i2c_ds3231_check_alarm_2(I2Cx);
			delayms(300);
			alarm2_state[1] = i2c_ds3231_check_alarm_2(I2Cx);
			if(alarm2_state[0] != 0){
				error_count++;
			}if (alarm2_state[1] != 1){
				error_count++;
			}
			if (error_count > 0)
			{
				char buffer[30];
				sprintf(buffer, "alarm2 day hr min %d %d\r\n", alarm2_state[0],alarm2_state[1]);
				uart_OutString(USART2, buffer);
			}
			if (result) return 1;
		}
	}

	return error_count;
}

/**********************************************************************************************/
/* Private functions                                                                          */
/**********************************************************************************************/
uint8_t _i2c_start_address(I2C_TypeDef* I2Cx, uint8_t address, uint8_t I2C_Direction){
	uint16_t timeout = 0;
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)){
		timeout++;
		if (timeout > DS3231_TIMEOUT){
			return 1;
		}
	}
	timeout = 0;

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	if (I2C_Direction == I2C_Direction_Transmitter){
		I2C_Send7bitAddress(I2Cx, address<<1, I2C_Direction_Transmitter);

		while(!(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR))){
			timeout++;
			if (timeout > DS3231_TIMEOUT){
				return 1;
			}
		}

	}else{
		I2C_Send7bitAddress(I2Cx, address<<1, I2C_Direction_Receiver);

		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			timeout++;
			if (timeout > DS3231_TIMEOUT){
				return 1;
			}
		}
	}

	I2C_ReadRegister(I2Cx, I2C_Register_SR2);

	return 0;
}

uint8_t _i2c_stop(I2C_TypeDef* I2Cx){
	uint16_t timeout = 0;
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE | I2C_FLAG_BTF)){
		timeout++;
		if (timeout > DS3231_TIMEOUT){
			return 1;
		}
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	return 0;
}

uint8_t _i2c_write(I2C_TypeDef* I2Cx, uint8_t byte){
	uint16_t timeout = 0;
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)){
		timeout++;
		if (timeout > DS3231_TIMEOUT){
			return 1;
		}
	}
	I2C_SendData(I2Cx, byte);
	return 0;
}

uint8_t _i2c_read(I2C_TypeDef* I2Cx, uint8_t ack, uint8_t * byte){
	uint16_t timeout = 0;
	if (ack == ACK){
		I2Cx->CR1 |= I2C_CR1_ACK;
	}else{
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
	}

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		timeout++;
		if (timeout > DS3231_TIMEOUT){
			return 1;
		}
	}

	*byte = I2C_ReceiveData(I2Cx);
	return 0;
}

