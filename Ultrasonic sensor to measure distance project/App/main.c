/*
 * Ultrasonic_sensor.c
 *
 * Created: 8/24/2024 1:06:00 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include "Ultrasonic_sensor.h"
#include "LCD.h"

int main(void)
{
	LCD_init();
	Ultrasonic_Init();
	unsigned int distance;
    while (1) 
    {
		distance=Ultrasonic_GetDistance_Polling(); // CM
		if (distance>80)
		{
			LCD_clr_screen();
			LCD_send_string("  No object");
			_delay_ms(1000);
		}
		else
		{
			LCD_move_cursor(1,1);
			LCD_send_string("distance=");
			LCD_send_data(distance/10+48);
			LCD_send_data(distance%10+48);
			LCD_send_string("cm");
		}
		
    }
}

