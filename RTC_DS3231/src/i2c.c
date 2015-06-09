/*
 * i2c.c
 *
 *  Created on: Jun 7, 2015
 *      Author: Dustin
 */

#include "stm32l1xx_i2c.h"
#include "string.h"
#include "i2c.h"

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

void i2c_ds3231_set_address(I2C_TypeDef* I2Cx, uint8_t address){
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while (I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2Cx, address);
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}
void i2c_ds3231_write_address(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value){
	//This needs to first set, then write
	//or combined?
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while (I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2Cx, address);
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));

	I2C_SendData(I2Cx, value);
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}
void i2c_ds3231_read_byte(I2C_TypeDef* I2Cx, uint8_t * data){

	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	*data = I2C_ReceiveData(I2Cx);

	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}

void i2c_ds3231_read_control_register(I2C_TypeDef* I2Cx, uint8_t * value){
	i2c_ds3231_set_address(I2Cx, DS3231_CONTROL_REG);
	i2c_ds3231_read_byte(I2Cx, value);
}

void i2c_ds3231_write_control_register(I2C_TypeDef* I2Cx, uint8_t value){
	i2c_ds3231_write_address(I2Cx, DS3231_CONTROL_REG, value);
}

void i2c_ds3231_dump_all_reg(I2C_TypeDef* I2Cx, uint8_t * data){
	uint8_t recv[20];
	uint8_t i;

	 I2C_AcknowledgeConfig(I2Cx, ENABLE);
	 while (I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));
	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2Cx, 0x00);  //set to beginning
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));

	// Now generate read, reading entire array
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for(i=0; i < 19; i++){
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
		recv[i] = I2C_ReceiveData(I2Cx);
		data[i] = recv[i];
	}
	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}
