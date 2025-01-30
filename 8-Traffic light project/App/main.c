/*
 * Traffic_light.c
 *
 * Created: 8/18/2024 9:41:33 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#include "macro_function.h"
#include "LCD.h"
#include "LED.h"
#include "SEVEN_SEGMENT.h"
#include "TIMER.h"
#define F_CPU 8000000UL
#include <util/delay.h>

volatile char counter;
int main(void)
{
    LCD_init();
	seven_seg_init_low_nibble();
	seven_seg_init_high_nibble();
	TIMER0_CTC_init_with_interrupt();
	LED_init('C',0);
	LED_init('C',1);
	LED_init('C',2);
	char count_red,count_yellow,count_green;

    while (1) 
    {
		count_green=30;
		count_yellow=5;
		count_red=20;
		LED_turn_on('C',0);
		LCD_clr_screen();
		LCD_send_string("      GO.....");
		while(count_green>0)
		{
			if (counter>=100)
			{
				counter=0;
				count_green--;
				seven_seg_write_low_nibble(count_green/10);
				seven_seg_write_high_nibble(count_green%10);
			}
			
		}
		 _delay_ms(10);  // Introduce a small delay to reduce CPU load
		LED_turn_off('C',0);
		LED_turn_on('C',1);
		LCD_clr_screen();
		LCD_send_string("     Ready");
		while(count_yellow>0)
		{
			if (counter>=100)
			{
				counter=0;
				count_yellow--;
				seven_seg_write_low_nibble(count_yellow/10);
				seven_seg_write_high_nibble(count_yellow%10);
			}
		}
		LED_turn_off('C',1);
		LED_turn_on('C',2);
		LCD_clr_screen();
		LCD_send_string("      STOP");
		 _delay_ms(10);  // Introduce a small delay to reduce CPU load
		while(count_red>0)
		{
			if (counter>=100)
			{
				counter=0;
				count_red--;
				seven_seg_write_low_nibble(count_red/10);
				seven_seg_write_high_nibble(count_red%10);
			}
		}
		LED_turn_off('C',2);
    }
}

// Timer0 Compare Match ISR
ISR(TIMER0_COMP_vect)
{
	counter++;
}

