/*
 * SPI_receive.c
 *
 * Created: 8/28/2024 5:07:10 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "LCD.h"
#include "SPI.h"
#define DUMMY_DATA 0xff

int main(void)
{
    LCD_init();
	SPI_slave_init();
	char x;
    while (1) 
    {
		x=SPI_slave_receive_and_transmit_data(DUMMY_DATA);
		LCD_send_data(x);
    }
}

