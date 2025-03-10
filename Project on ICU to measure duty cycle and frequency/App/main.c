/*
 * ICU_measure_duty_cycle_and_frequency.c
 *
 * Created: 8/17/2024 7:48:30 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#include "LCD.h"
#include "TIMER.h"
#define F_CPU 8000000UL

int main(void)
{
    LCD_init();
	unsigned short first_val,socend_val,third_val,on_period,period,duty_cycle,frequency;
	char prescaler=1;
	
	TIMER1_ICU_init();
	first_val=TIMER1_ICU_raising();
	socend_val=TIMER1_ICU_failing();
	third_val=TIMER1_ICU_raising();
	TCCR1B=0;
	if (first_val<socend_val && socend_val<third_val)
	{
		on_period=socend_val-first_val;
		period=third_val-first_val;
		duty_cycle=((float)on_period/period)*100;
		frequency=F_CPU/((float)prescaler*period);
		LCD_send_string("Duty=");
		if (duty_cycle<10)
		{
			LCD_send_data(duty_cycle+48);
		}
		else if (duty_cycle<100)
		{
			LCD_send_data(duty_cycle/10+48);
			LCD_send_data(duty_cycle%10+48);
		}
		LCD_send_data('%');
		LCD_move_cursor(2,1);
		LCD_send_string("Frequency=");
		if (frequency<10)
		{
			LCD_send_data(frequency+48);
		}
		else if (frequency<100)
		{
			LCD_send_data(frequency/10+48);
			LCD_send_data(frequency%10+48);
		}
		else if (frequency<1000)
		{
			LCD_send_data(frequency/100+48);
			LCD_send_data((frequency%100)/10+48);
			LCD_send_data(frequency%10+48);
		}
		else if (frequency<10000)
		{
			LCD_send_data(frequency/1000+48);
			LCD_send_data((frequency%1000)/100+48);
			LCD_send_data((frequency%100)/10+48);
			LCD_send_data(frequency%10+48);
		}
		LCD_send_string("HZ");
	}
	else
	{
		LCD_clr_screen();
		LCD_send_string("Out of Range!!");
	}
	
}