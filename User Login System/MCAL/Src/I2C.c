/*
 * I2C.c
 *
 * Created: 9/1/2024 3:00:44 PM
 *  Author: Ahmed Ragab
 */ 

#include <avr/io.h>
#include "macro_function.h"
#define F_CPU 8000000UL
#define prescaler 1

void I2C_master_init(unsigned long scl_clock)
{
	// Calculate TWBR based on the formula for desired SCL frequency
	TWBR = ((F_CPU / scl_clock) - 16) / (2 * prescaler);
	// Set the prescaler bits in TWSR (TWI Status Register)
	switch (prescaler)
	{
		case 1:
		TWSR = 0x00;
		break;
		case 4:
		TWSR = 0x01;
		break;
		case 16:
		TWSR = 0x02;
		break;
		case 64:
		TWSR = 0x03;
		break;
		default:
		TWSR = 0x00;  // Default to no prescaler if an invalid value is provided
		break;
	}
}

void I2C_set_address(unsigned char address)
{
	// Set device address with general call recognition disabled by default
	TWAR = (address << 1);
	// SET_BIT(TWAR, TWGCE);  // Enable general call recognition
}

void I2C_start(void)
{
	// Clear flag, enable I2C, and send start condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	// Wait for the start condition to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if start condition has been transmitted successfully
	while ((TWSR & 0xF8) != 0x08);
}

void I2C_repeated_start(void)
{
	// Clear flag, enable I2C, and send start condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	// Wait for the repeated start condition to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if repeated start condition has been transmitted successfully
	while ((TWSR & 0xF8) != 0x10);
}

void I2C_stop(void)
{
	// Clear flag, enable I2C, and send stop condition
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void I2C_master_send_address_transmitter_mode(unsigned char address)
{
	// Load the slave address and write mode
	TWDR=(address<<1);
	//clear flag and enable I2C 
	TWCR=(1<<TWINT) | (1<<TWEN);
	// Wait for the address to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if SLA+W has been transmitted and ACK received
	while((TWSR&0XF8)!=0X18);
}

void I2C_master_send_address_receiver_mode(unsigned char address)
{
	// Load the slave address and read mode
	TWDR=(address<<1) | 1;
	//clear flag and enable I2C
	TWCR=(1<<TWINT) | (1<<TWEN);
	// Wait for the address to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if SLA+R has been transmitted and ACK received
	while((TWSR&0XF8)!=0X40);
}

void I2C_master_transmit_data(char data)
{
	// Load data into the data register
	TWDR=data;
	//clear flag and enable I2C
	TWCR=(1<<TWINT) | (1<<TWEN);
	// Wait for data to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if data has been transmitted and ACK received
	while((TWSR&0XF8)!=0X28);
}

char I2C_master_receive_data(void)
{
	//clear flag and enable I2C and enable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	//wait for data to  be received
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if data has been received and ACK sent
	while((TWSR&0XF8)!=0X50);
	// Return the received data
	return TWDR;
}

char I2C_master_receive_data_with_not_ACK(void)
{
	//clear flag and enable I2C and disable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN);
	//wait for data to  be received
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if data has been received and not ACK sent
	while((TWSR&0XF8)!=0X58);
	// Return the received data
	return TWDR;
}

void I2C_slave_transmit_data(char data)
{
	//clear flag and enable I2C and enable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// Wait for address match
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if own SLA+R has been received
	while((TWSR&0XF8)!=0XA8);
	// Load data to data register
	TWDR=data;
	//clear flag and enable I2C and enable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// Wait for data to be transmitted
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if data has been transmitted and ACK received
	while((TWSR&0XF8)!=0XB8);
}

char I2C_slave_receive_data(void)
{
	//clear flag and enable I2C and enable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// Wait for address match
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if own SLA+W has been received
	while((TWSR&0XF8)!=0X60);
	//clear flag and enable I2C and enable acknowledgment
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// Wait for data to be received
	while(READ_BIT(TWCR,TWINT)==0);
	// Check if data has been received and ACK sent
	while((TWSR&0XF8)!=0X80);
	// Return the received data
	return TWDR;
}