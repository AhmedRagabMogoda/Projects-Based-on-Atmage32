/*
 * write_arabic_and_english_on_LCD.c
 *
 * Created: 8/14/2024 9:23:34 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "LCD.h"

int main(void)
{
    char create_char[8][8] = {
	    {
		    0b00000111,
		    0b00000100,
		    0b00001111,
		    0b00000110,        //letter of Alif (A)
		    0b00000110,
		    0b00000110,
		    0b00000110,
		    0b00000110
	    },
	    {
		    0b00010000,
		    0b00011000,
		    0b00001100,        //letter of 7A (H)
		    0b00000110,
		    0b00000011,
		    0b00011111,
		    0b00011111,
		    0b00000000
	    },
	    {
		    0b00010000,
		    0b00011000,
		    0b00000110,      //letter of Gam (G)
		    0b00000011,
		    0b00011111,
		    0b00011111,
		    0b00001100,
		    0b00001100
	    },
	    {
		    0b00000000,
		    0b00000011,
		    0b00000011,
		    0b00000011,
		    0b00011111,     //letter of Dal (D)
		    0b00011111,
		    0b00000000,
		    0b00000000
	    },
	    {
		    0b00000000,
		    0b00011111,
		    0b00010011,
		    0b00010011,    //letter of Wow
		    0b00011111,
		    0b00000011,
		    0b00011111,
		    0b00000000
	    },
	    {
		    0b00000000,
		    0b00000000,
		    0b00001110,
		    0b00001001,    //letter of Mem
		    0b00011001,
		    0b00011001,
		    0b00000110,
		    0b00000000
	    },
	    {

		    0b00000000,
		    0b00001110,
		    0b00010001,      //letter of Ha
		    0b00010001,
		    0b00010001,
		    0b00001110,
		    0b00000000,
		    0b00000000
	    },
	    {
		    0b00000000,
		    0b00010001,
		    0b00011011,
		    0b00011111,
		    0b00011111,       //letter of Heart (?)
		    0b00001110,
		    0b00000100,
		    0b00000000
	    },
    };
    
    LCD_init();
    LCD_create_char(create_char);
	_delay_ms(100);
	LCD_send_data('A');
	_delay_ms(100);
	LCD_send_data('h');
	_delay_ms(100);
	LCD_send_data('m');
	_delay_ms(100);
	LCD_send_data('e');
	_delay_ms(100);
	LCD_send_data('d');
	_delay_ms(100);
	LCD_send_data(' ');
	_delay_ms(100);
	LCD_send_data('M');
	_delay_ms(100);
	LCD_send_data('o');
	_delay_ms(100);
	LCD_send_data('g');
	_delay_ms(100);
	LCD_send_data('o');
	_delay_ms(100);
	LCD_send_data('d');
	_delay_ms(100);
	LCD_send_data('a');
	_delay_ms(4000);
	LCD_clr_screen();
	LCD_move_cursor(1,16);
	LCD_send_cmd(0x04);
	_delay_ms(100);
	LCD_send_data(0);
	_delay_ms(100);
	LCD_send_data(1);
	_delay_ms(100);
	LCD_send_data(5);
	_delay_ms(100);
	LCD_send_data(3);
	_delay_ms(100);
	LCD_send_data(0x20);
	_delay_ms(100);
	LCD_send_data(5);
	_delay_ms(100);
	LCD_send_data(2);
	_delay_ms(100);
	LCD_send_data(4);
	_delay_ms(100);
	LCD_send_data(3);
	_delay_ms(100);
	LCD_send_data(6);
	_delay_ms(100);
	LCD_send_data(0x20);
	_delay_ms(100);
	LCD_send_data(7);
	_delay_ms(4000);
	LCD_clr_screen();
	_delay_ms(100);
    LCD_send_string("Ahmed Mogoda");
    _delay_ms(1000);
    LCD_move_cursor(2,16);
    LCD_send_cmd(0x04);
    LCD_send_data(0);
    LCD_send_data(1);
    LCD_send_data(5);
    LCD_send_data(3);
    LCD_send_data(0x20);
    LCD_send_data(5);
    LCD_send_data(2);
    LCD_send_data(4);
    LCD_send_data(3);
    LCD_send_data(6);
    LCD_send_data(0x20);
    LCD_send_data(7);
}

