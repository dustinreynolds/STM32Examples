/*
 * spi.h
 *
 *  Created on: May 9, 2015
 *      Author: Dustin
 */

#ifndef SPI_H_
#define SPI_H_

#define SPI2_BUFFER_SIZE 260

void spi_SPI2_Configuration(void);
uint8_t spi_send(uint8_t data);
uint8_t __inline__ spi_send_buffer(uint32_t size);
#endif /* SPI_H_ */
