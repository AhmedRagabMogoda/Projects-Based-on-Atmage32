/*
 * LCD.c
 *
 * Created: 7/24/2024 2:01:29 AM
 *  Author: Ahmed Ragab
 */ 
#define F_CPU 8000000UL
#include <util/delay.h>
#include "LCD.h"
#include "DIO.h"


void LCD_init(void)
{
	_delay_ms(200);
	#if defined eight_bits_mode
	DIO_set_port_dir(PORT_1,0xff);
	DIO_set_pin_dir(PORT_2,EN,1);
	DIO_set_pin_dir(PORT_2,RW,1);
	DIO_set_pin_dir(PORT_2,RS,1);
	DIO_write_pin(PORT_2,RW,0);
	
	LCD_send_cmd(EIGHT_BITS);  //8_bits_mode
	_delay_ms(1);
	LCD_send_cmd(CLR_SCREEN);  //clear screen
	_delay_ms(10);
	LCD_send_cmd(CURSOR_Off_DISPLAY_ON); //curson on, display on
	_delay_ms(1);
	LCD_send_cmd(ENTRY_MODE);  //sift curson to right
	_delay_ms(1);
	
	#elif defined four_bits_mode
	DIO_set_pin_dir(PORT_1,4,1);
	DIO_set_pin_dir(PORT_1,5,1);
	DIO_set_pin_dir(PORT_1,6,1);
	DIO_set_pin_dir(PORT_1,7,1);
	DIO_set_pin_dir(PORT_2,EN,1);
	DIO_set_pin_dir(PORT_2,RW,1);
	DIO_set_pin_dir(PORT_2,RS,1);
	DIO_write_pin(PORT_2,RW,0);	
	
	LCD_send_cmd(RETURN_HOME); //4_bits_mode //return home
	_delay_ms(10);
	LCD_send_cmd(FOUR_BITS);  //4_bits_mode
	_delay_ms(1);
	LCD_send_cmd(CLR_SCREEN);  //clear screen
	_delay_ms(10);
	LCD_send_cmd(CURSOR_Off_DISPLAY_ON); //curson on, display on
	_delay_ms(1);
	LCD_send_cmd(ENTRY_MODE);  //sift curson to right
	_delay_ms(1);
	#endif
}

static void enable(void)
{
	DIO_write_pin(PORT_2,EN,1);
	_delay_ms(2);
	DIO_write_pin(PORT_2,EN,0);
	_delay_ms(2);
}

void LCD_send_cmd(char cmd)
{
	DIO_write_pin(PORT_2,RS,0);
	#if defined eight_bits_mode
	DIO_write_port(PORT_1,cmd);
	enable();
	#elif defined four_bits_mode
	DIO_write_high_nibble(PORT_1,cmd>>4);
	enable();
	DIO_write_high_nibble(PORT_1,cmd);
	enable();
	#endif
	_delay_ms(1);
}

void LCD_send_data(char data)
{
	DIO_write_pin(PORT_2,RS,1);
	#if defined eight_bits_mode
	DIO_write_port(PORT_1,data);
	enable();
	
	#elif defined four_bits_mode
	DIO_write_high_nibble(PORT_1,data>>4);
	enable();
	DIO_write_high_nibble(PORT_1,data);
	enable();
	#endif
	
}

void LCD_send_string(char* string)
{
	while((*string!='\0'))
	{
		LCD_send_data(*string);
		string++;
	}
}

void LCD_clr_screen(void)
{
	LCD_send_cmd(CLR_SCREEN);
	_delay_ms(10);
}

void LCD_move_cursor(char row,char coloum)
{
	char data=0;
	if (row>2 || row<1 || coloum>16 || coloum<1)
	{
		data=0x80;
	}
	else if (row==1)
	{
		data=(0x80+coloum-1);
	}
	else if (row==2)
	{
		data=(0xC0+coloum-1);
	}
	LCD_send_cmd(data);
	_delay_ms(1);
}
