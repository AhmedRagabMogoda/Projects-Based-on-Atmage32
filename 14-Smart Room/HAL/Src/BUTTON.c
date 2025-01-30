/*
 * button.c
 *
 * Created: 5/10/2024 10:38:20 PM
 *  Author: Ahmed Ragab
 */ 

#include "BUTTON.h"
#include "DIO.h"

void BUTTON_init(unsigned char port,unsigned char pin_number)
{
	DIO_set_pin_dir(port,pin_number,0);
}

void BUTTON_init_with_pull_up(unsigned char port,unsigned char pin_number)
{
	DIO_set_pin_dir(port,pin_number,0); // input
	DIO_write_pin(port,pin_number,1); //pull up
}

unsigned char BUTTON_read(unsigned char port,unsigned char pin_number)
{
	unsigned char ret_value;
	ret_value=DIO_read_pin(port,pin_number);
	return ret_value;
}
