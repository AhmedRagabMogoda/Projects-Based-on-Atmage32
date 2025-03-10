/*
 * SEVEN_SEGMENT.c
 *
 * Created: 6/30/2024 10:01:44 PM
 *  Author: Ahmed Ragab
 */ 

#include "SEVEN_SEGMENT.h"

void seven_seg_init_port(void)
{
	DIO_set_port_dir(seven_seg_port,0xff);
}

void seven_seg_write_port(unsigned char number)
{
	#ifdef CC
	unsigned char display_number[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x47,0x7F,0x6F};
	DIO_write_port(seven_seg_port,display_number[number]);
	#else
	unsigned char display_number[]={~0x3F,~0x06,~0x5B,~0x4F,~0x66,~0x6D,~0x7D,~0x47,~0x7F,~0x6F};
	DIO_write_port(seven_seg_port,display_number[number]);	
	#endif
}

void seven_seg_init_low_nibble(void)
{
	DIO_set_pin_dir(seven_seg_port,0,1);
	DIO_set_pin_dir(seven_seg_port,1,1);
	DIO_set_pin_dir(seven_seg_port,2,1);
	DIO_set_pin_dir(seven_seg_port,3,1);
}

void seven_seg_init_high_nibble(void)
{
	DIO_set_pin_dir(seven_seg_port,4,1);
	DIO_set_pin_dir(seven_seg_port,5,1);
	DIO_set_pin_dir(seven_seg_port,6,1);
	DIO_set_pin_dir(seven_seg_port,7,1);
}

void seven_seg_write_low_nibble(unsigned char number)
{
DIO_write_low_nibble(seven_seg_port,number);
}

void seven_seg_write_high_nibble(unsigned char number)
{
	DIO_write_high_nibble(seven_seg_port,number);
}

void seven_seg_init_nibble(unsigned char pin_start)
{
	DIO_set_pin_dir(seven_seg_port,pin_start,1);
	DIO_set_pin_dir(seven_seg_port,pin_start+1,1);
	DIO_set_pin_dir(seven_seg_port,pin_start+2,1);
	DIO_set_pin_dir(seven_seg_port,pin_start+3,1);
}

void seven_seg_write_nibble(unsigned char pin_start,unsigned char number)
{
	DIO_write_pin(seven_seg_port,pin_start  ,READ_BIT(number,0));
	DIO_write_pin(seven_seg_port,pin_start+1,READ_BIT(number,1));
	DIO_write_pin(seven_seg_port,pin_start+2,READ_BIT(number,2));
	DIO_write_pin(seven_seg_port,pin_start+3,READ_BIT(number,3));
}
