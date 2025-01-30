/*
 * SPI.c
 *
 * Created: 8/28/2024 5:07:10 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "SPI.h"
#include "KEYBAD.h"

int main(void)
{
    SPI_master_init();
	KEYBAD_init();
	_delay_ms(500);
	char key;
    while (1) 
    {
		do 
		{
			key=KEYBAD_check_press();
		} while (0xff==key);
		_delay_ms(250);
		SPI_master_transmit_and_receive_data(key);
    }
}

