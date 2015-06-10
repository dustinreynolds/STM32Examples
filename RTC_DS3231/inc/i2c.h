/*
 * i2c.h
 *
 *  Created on: Jun 7, 2015
 *      Author: Dustin
 */

#ifndef I2C_H_
#define I2C_H_

#define ACK	1
#define NACK 2

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


void i2c_init(void);
void i2c_temp_write_reg(I2C_TypeDef* I2Cx, uint8_t address, uint8_t value);
void i2c_temp_read_register(I2C_TypeDef* I2Cx, uint8_t address, uint8_t * value);


void i2c_ds3231_read_control_register(I2C_TypeDef* I2Cx, uint8_t * value);
void i2c_ds3231_write_control_register(I2C_TypeDef* I2Cx, uint8_t value);
void i2c_ds3231_dump_all_reg(I2C_TypeDef* I2Cx, uint8_t * data);

#endif /* I2C_H_ */
