/*
 * Real_time_clock.c
 *
 * Created: 8/22/2024 12:48:38 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "macro_function.h"
#include "LCD.h"
#include "KEYBAD.h"
#include "SEVEN_SEGMENT.h"
#include "TIMER.h"

void display_clock(void);

volatile unsigned char seconds_counter=0;
unsigned char minutes_counter=0,hours_counter=0;
unsigned char ret_val,count,first_digital,second_digital;

int main(void)
{
    LCD_init();
	KEYBAD_init();
	seven_seg_init_port();
	TIMER2_normal_init_with_interrupt();
	SET_BIT(DDRC,PC0);
	SET_BIT(DDRC,PC1);
	SET_BIT(DDRC,PC2);
	SET_BIT(DDRC,PC3);
	SET_BIT(DDRC,PC4);
	SET_BIT(DDRC,PC5);
	LCD_send_string("   press A");
	LCD_move_cursor(2,1);
	LCD_send_string("   to set clock");
    while (1) 
    {
		ret_val=KEYBAD_check_press();
		if ('A'==ret_val)
		{
			LCD_clr_screen();
			LCD_send_string(" Hour=--");
			LCD_move_cursor(1,7);
			do
			{
				ret_val=KEYBAD_check_press();
				if ('0'==ret_val || '1'==ret_val || '2'==ret_val || '3'==ret_val || '4'==ret_val || '5'==ret_val || '6'==ret_val || '7'==ret_val || '8'==ret_val || '9'==ret_val)
				{
					if (0==count)
					{
						first_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
						_delay_ms(200);
					}
					else
					{
						second_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
					}
				}
				display_clock();
			} while (count<2);
			_delay_ms(200);
			if (24>(((first_digital-48)*10)+(second_digital-48)))
			{
				hours_counter=(first_digital-48)*10+(second_digital-48);
			}
			first_digital=second_digital='0';
			count=0;
			LCD_clr_screen();
			LCD_send_string("Minute=--");
			LCD_move_cursor(1,8);
			do
			{
				ret_val=KEYBAD_check_press();
				if ('0'==ret_val || '1'==ret_val || '2'==ret_val || '3'==ret_val || '4'==ret_val || '5'==ret_val || '6'==ret_val || '7'==ret_val || '8'==ret_val || '9'==ret_val)
				{
					if (0==count)
					{
						first_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
						_delay_ms(200);
					}
					else
					{
						second_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
					}
				}
				display_clock();
			} while (count<2);
			_delay_ms(200);
			if (60>((first_digital-48)*10))
			{
				minutes_counter=(first_digital-48)*10+(second_digital-48);
			}
			first_digital=second_digital='0';
			count=0;
			LCD_clr_screen();
			LCD_send_string("Second=--");
			LCD_move_cursor(1,8);
			do
			{
				ret_val=KEYBAD_check_press();
				if ('0'==ret_val || '1'==ret_val || '2'==ret_val || '3'==ret_val || '4'==ret_val || '5'==ret_val || '6'==ret_val || '7'==ret_val || '8'==ret_val || '9'==ret_val)
				{
					if (0==count)
					{
						first_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
						_delay_ms(200);
					}
					else
					{
						second_digital=ret_val;
						LCD_send_data(ret_val);
						count++;
					}
				}
				display_clock();
			} while (count<2);
			_delay_ms(200);
			if (60>(first_digital-48)*10)
			{
				seconds_counter=(first_digital-48)*10+(second_digital-48);
			}
			first_digital=second_digital='0';
			count=0;
			LCD_clr_screen();
			LCD_send_string("   press A");
			LCD_move_cursor(2,1);
			LCD_send_string("   to set clock");
		}
		display_clock();
    }
}

ISR(TIMER2_OVF_vect)
{
	seconds_counter++;
}

void display_clock(void)
{
	CLR_BIT(PORTC,PC0);
	SET_BIT(PORTC,PC1);
	SET_BIT(PORTC,PC2);
	SET_BIT(PORTC,PC3);
	SET_BIT(PORTC,PC4);
	SET_BIT(PORTC,PC5);
	seven_seg_write_port(hours_counter/10);
	_delay_ms(5);
	SET_BIT(PORTC,PC0);
	CLR_BIT(PORTC,PC1);
	SET_BIT(PORTC,PC2);
	SET_BIT(PORTC,PC3);
	SET_BIT(PORTC,PC4);
	SET_BIT(PORTC,PC5);
	seven_seg_write_port(hours_counter%10);
	_delay_ms(5);
	SET_BIT(PORTC,PC0);
	SET_BIT(PORTC,PC1);
	CLR_BIT(PORTC,PC2);
	SET_BIT(PORTC,PC3);
	SET_BIT(PORTC,PC4);
	SET_BIT(PORTC,PC5);
	seven_seg_write_port(minutes_counter/10);
	_delay_ms(5);
	SET_BIT(PORTC,PC0);
	SET_BIT(PORTC,PC1);
	SET_BIT(PORTC,PC2);
	CLR_BIT(PORTC,PC3);
	SET_BIT(PORTC,PC4);
	SET_BIT(PORTC,PC5);
	seven_seg_write_port(minutes_counter%10);
	_delay_ms(5);
	SET_BIT(PORTC,PC0);
	SET_BIT(PORTC,PC1);
	SET_BIT(PORTC,PC2);
	SET_BIT(PORTC,PC3);
	CLR_BIT(PORTC,PC4);
	SET_BIT(PORTC,PC5);
	seven_seg_write_port(seconds_counter/10);
	_delay_ms(5);
	SET_BIT(PORTC,PC0);
	SET_BIT(PORTC,PC1);
	SET_BIT(PORTC,PC2);
	SET_BIT(PORTC,PC3);
	SET_BIT(PORTC,PC4);
	CLR_BIT(PORTC,PC5);
	seven_seg_write_port(seconds_counter%10);
	_delay_ms(5);
	if (60==seconds_counter)
	{
		seconds_counter=0;
		minutes_counter++;
	}
	if (60==minutes_counter)
	{
		minutes_counter=0;
		hours_counter++;
	}
	if (24==hours_counter)
	{
		hours_counter=0;
	}
}