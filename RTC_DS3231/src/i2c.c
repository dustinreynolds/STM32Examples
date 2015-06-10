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

void _i2c_start_address(I2C_TypeDef* I2Cx, uint8_t address, uint8_t I2C_Direction){
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	if (I2C_Direction == I2C_Direction_Transmitter){
		I2C_Send7bitAddress(I2Cx, address<<1, I2C_Direction_Transmitter);

		while(!(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR)));

	}else{
		I2C_Send7bitAddress(I2Cx, address<<1, I2C_Direction_Receiver);

		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

	I2C_ReadRegister(I2Cx, I2C_Register_SR2);
}

void _i2c_stop(I2C_TypeDef* I2Cx){
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE | I2C_FLAG_BTF));
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void _i2c_write(I2C_TypeDef* I2Cx, uint8_t byte){
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));
	I2C_SendData(I2Cx, byte);
}

void _i2c_read(I2C_TypeDef* I2Cx, uint8_t ack, uint8_t * byte){

	if (ack == ACK){
		I2Cx->CR1 |= I2C_CR1_ACK;
	}else{
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
	}

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

	*byte = I2C_ReceiveData(I2Cx);
}

void i2c_temp_set_address(I2C_TypeDef* I2Cx, uint8_t address){
	_i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Transmitter);
	_i2c_write(I2Cx, address);
	_i2c_stop(I2Cx);
}

void i2c_temp_write_reg(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value){
	_i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Transmitter);
	_i2c_write(I2Cx, address);
	_i2c_write(I2Cx, value);
	_i2c_stop(I2Cx);
}

void i2c_temp_read_register(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * value){
	i2c_temp_set_address(I2Cx, address);

	_i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Receiver);
	_i2c_read(I2Cx, NACK, value);
}

void i2c_temp_read_registers(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * data, uint8_t count){
	i2c_temp_set_address(I2Cx, address);

	_i2c_start_address(I2Cx, DS3231_I2C_ADDR,I2C_Direction_Receiver);
	while (--count > 0)
		_i2c_read(I2Cx, ACK, data++);
	_i2c_read(I2Cx, NACK, data);
}

/* I2C Set Address conforms to Manual */
void i2c_ds3231_set_address(I2C_TypeDef* I2Cx, uint8_t address){
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2Cx, address);

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE | I2C_FLAG_BTF));

//	I2Cx->CR1 |= I2C_CR1_START;
//	while (!(I2Cx->SR1 & I2C_SR1_SB));
//
//	I2Cx->CR1 |= I2C_CR1_ACK;
//
//	I2Cx->DR = (DS3231_I2C_ADDR<<1) & ~I2C_OAR1_ADD0;
//	while(!(I2Cx->SR1 & I2C_SR1_ADDR));
//
//	I2Cx->SR2;
//
//	//send address here
//	while (!(I2Cx->SR1 & I2C_SR1_TXE));
//
//	I2Cx->DR = address;
//
//	I2Cx->CR1 |= I2C_CR1_STOP;
//	while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF))));
}

void i2c_ds3231_write_address(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value){
	//This needs to first set, then write
	//or combined?
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Transmitter);
	//while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	while(!(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR)));
	I2C_ReadRegister(I2Cx, I2C_Register_SR2);

	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));
	I2C_SendData(I2Cx, address);

	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));
	I2C_SendData(I2Cx, value);

	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE | I2C_FLAG_BTF));
	I2C_GenerateSTOP(I2Cx, ENABLE);

}
void i2c_ds3231_read_byte(I2C_TypeDef* I2Cx, uint8_t * data){
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	I2C_Send7bitAddress(I2Cx, DS3231_I2C_ADDR<<1, I2C_Direction_Receiver);

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);

	I2C_GenerateSTOP(I2Cx, ENABLE);

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

	data[0] = I2C_ReceiveData(I2Cx);

	asm("nop");
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
