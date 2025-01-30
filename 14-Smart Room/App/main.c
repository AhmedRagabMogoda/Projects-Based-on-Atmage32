/*
 * Smart_Room.c
 *
 * Created: 5/12/2024 3:14:11 PM
 * Author : Ahmed Ragab
 */ 

#define  F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include "ADC.h"
#include "LCD.h"
#include "BUTTON.h"
#include "LED.h"


char read_val;
unsigned short temperatue;

int main(void)
{
	LCD_init();
	ADC_init();
	LED_init('C',0);
	LED_init('C',1);
	LED_init('D',0);
	LED_init('D',1);		
	BUTTON_init('D',2);
	BUTTON_init('D',3);	
	char led_value=0;
	LCD_send_string("temperature=");
	
	while(1)
	{
		if((1==BUTTON_read('D',2)) && (led_value==0))
		{
			LED_toggle('C',0);
			led_value=1;
		}
		else if(0==BUTTON_read('D',2))
		{
			led_value=0;
		}
		else 
		{
			
		}
		

		
		if(1==BUTTON_read('D',3))
		{
			LED_turn_on('C',1);
			_delay_ms(500);
		}
		else
		{
			LED_turn_off('C',1);
		}		
		
		
		
		
		temperatue=(ADC_u16Read()*0.25);
		
		if(temperatue<10)
		{
			LCD_move_cursor(1,13);
			LCD_send_data(0x30);
			LCD_send_data((temperatue%10)+48);
			LCD_send_data(0xDF);
			LCD_send_data(0x43);
			
		}
		else if( temperatue<100)
		{
			LCD_move_cursor(1,13);
			LCD_send_data((temperatue/10)+48);
			LCD_send_data((temperatue%10)+48);
			LCD_send_data(0XDF);
			LCD_send_data(0X43);			
			
		}
		else
		{
			
		}
		
		
		if(temperatue<30)
		{
			LED_turn_on('D',1);
		}
			else
		{
			LED_turn_off('D',1);
		}
						
		if (temperatue>=30)
		{
			LED_turn_on('D',0);
		}
		else
		{
			LED_turn_off('D',0);
		}			
		
	}
}