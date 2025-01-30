/*
 * CFile1.c
 *
 * Created: 8/28/2024 5:13:34 PM
 *  Author: Ahmed Ragab
 */
 
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "macro_function.h"


void SPI_master_init(void)
{
	// SS,MOSI,SCK AS output and MISO AS input
	SET_BIT(DDRB,PB4);  // SS as output
	SET_BIT(DDRB,PB5);  // MOSI as output
	SET_BIT(DDRB,PB7);  // SCK as output
	SET_BIT(PORTB,PB4);  // not active slave
	// Enable SPI
	SET_BIT(SPCR,SPE);
	//Enable master mode
	SET_BIT(SPCR,MSTR);
	// choose clock = F_cpu/16
	SET_BIT(SPCR,SPR0);
	CLR_BIT(SPCR,SPR1);
}

void SPI_slave_init(void)
{
	//SS,MOSI,SCK AS input and MISO AS output
	SET_BIT(DDRB,PB6);  // MISO as output
	// Enable SPI
	SET_BIT(SPCR,SPE);
	//Enable slave mode
	CLR_BIT(SPCR,MSTR);
}

unsigned char SPI_master_transmit_and_receive_data(unsigned char data)
{
	 // active slave
	CLR_BIT(PORTB,PB4);
	//put data on SPDR
	SPDR=data;
	// wait data transfer is complete
	while(READ_BIT(SPSR,SPIF)==0);
	//read data on SPDR
	return SPDR;	
}

void SPI_master_transmit_string(unsigned char *ptr)
{
	while('\0'!=(*ptr))
	{
		SPI_master_transmit_and_receive_data(*ptr);
		_delay_ms(300);
		ptr++;
	}
}

unsigned char SPI_slave_receive_and_transmit_data(unsigned char data)
{
	//put data on SPDR
	SPDR=data;
	// wait data transfer is complete
	while(READ_BIT(SPSR,SPIF)==0);
	//read data on SPDR
	return SPDR;
}


void SPI_master_init_with_interrupt(void)
{
	// SS,MOSI,SCK AS output and MISO AS input
	SET_BIT(DDRB,PB4);  // SS as output
	SET_BIT(DDRB,PB5);  // MOSI as output
	SET_BIT(DDRB,PB7);  // SCK as output
	SET_BIT(PORTB,PB4);  // not active slave
	// Enable SPI
	SET_BIT(SPCR,SPE);
	//Enable master mode
	SET_BIT(SPCR,MSTR);
	// choose clock = F_cpu/16
	SET_BIT(SPCR,SPR0);
	CLR_BIT(SPCR,SPR1);
	//glable interrupt
	sei();
	//SPI Interrupt
	SET_BIT(SPCR,SPIE);
	//accurs interrupt when data transfer is complete
}