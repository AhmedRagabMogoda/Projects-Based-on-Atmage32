/*
 * LED.c
 *
 * Created: 5/10/2024 4:34:35 PM
 *  Author: Ahmed Ragab
 */ 

#include "LED.h"
#include "DIO.h"

void LED_init(unsigned char port,unsigned char pin_number)
{
	DIO_set_pin_dir(port,pin_number,1);
}

void LED_turn_on(unsigned char port,unsigned char pin_number)
{
	DIO_write_pin(port,pin_number,1);
}

void LED_turn_off(unsigned char port,unsigned char pin_number)
{
	DIO_write_pin(port,pin_number,0);
}

void LED_toggle(unsigned char port,unsigned char pin_number)
{
	DIO_toggle_pin(port,pin_number);
}