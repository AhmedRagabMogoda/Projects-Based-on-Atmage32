/*
 * Temperature_Meter.c
 *
 * Created: 8/9/2024 6:54:58 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#include "ADC.h"
#include "LCD.h"

int main(void)
{
    LCD_init();
	ADC_init();
	unsigned short volt;
	unsigned short temperature;
	
	LCD_send_string("Temperature=");
    while (1) 
    {
		volt= ADC_u16read()*resolution*1000; //( mV )                   
		if (volt>=1000)
		{
			temperature=(volt-1000)/10;      //1degree=10mv
			if (temperature<10)
			{
				LCD_move_cursor(1,13);
				LCD_send_data(' ');
				LCD_send_data(temperature+48);
				LCD_send_data(0xDF);
				LCD_send_data('C');
			}
			else if(temperature<100)
			{
				LCD_move_cursor(1,13);
				LCD_send_data(temperature/10+48);
				LCD_send_data(temperature%10+48);
				LCD_send_data(0xDF);
				LCD_send_data('C');
			}
		}
		else
		{
			temperature=(1000-volt)/10;
			if (temperature<10)
			{
				LCD_move_cursor(1,13);
				LCD_send_data('-');
				LCD_send_data(temperature+48);
				LCD_send_data(0xDF);
				LCD_send_data('C');
			}
			else if(temperature<100)
			{
				LCD_move_cursor(1,13);
				LCD_send_data('-');
				LCD_send_data(temperature/10+48);
				LCD_send_data(temperature%10+48);
				LCD_send_data(0xDF);
				LCD_send_data('C');
			}
		}
		
		
    }
}

