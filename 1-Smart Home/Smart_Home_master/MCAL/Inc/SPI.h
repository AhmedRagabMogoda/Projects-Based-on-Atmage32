/*
 * SPI.h
 *
 * Created: 8/28/2024 5:14:07 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef SPI_H_
#define SPI_H_

void SPI_master_init(void);

void SPI_slave_init(void);

unsigned char SPI_master_transmit_and_receive_data(unsigned char data);

void SPI_master_transmit_string(unsigned char *ptr);

unsigned char SPI_slave_receive_and_transmit_data(unsigned char data);

void SPI_master_init_with_interrupt(void);

#endif /* SPI_H_ */