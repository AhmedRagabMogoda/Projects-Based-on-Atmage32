/*
 * garage project.c
 *
 * Created: 8/28/2024 11:32:20 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include "LCD.h"
#include "MOTOR.h"
#include "macro_function.h"

int main(void)
{
  LCD_init();
  unsigned char IR_1=0,IR_2=0;
  unsigned char flag_1=0,flag_2=0;
  unsigned char count=0;
  LCD_send_string("there are 0 cars");
  LCD_move_cursor(2,1);
  LCD_send_string("garage has space");
    while (1) 
    {
		IR_1=READ_BIT(PIND,PD0);
		if (1==IR_1 && 0==flag_1 && count<3)
		{
			count++;
			Servo_motor_with_TIMER1_fast_pwm_ICRA_wave_A(90);
			_delay_ms(1000);
			Servo_motor_with_TIMER1_fast_pwm_ICRA_wave_A(0);
			LCD_move_cursor(1,11);
			LCD_send_data(count+48);
			flag_1=1;
			if (3==count)
			{
				LCD_clr_screen();
				LCD_send_string("there are 3 cars");
				LCD_move_cursor(2,1);
				LCD_send_string("garage is full");
			}
		}else if (0==IR_1)
		{
			flag_1=0;
		}
		
		IR_2=READ_BIT(PIND,PD1);
		if (1==IR_2 && 0==flag_2 && count>0)
		{
			count--;
			Servo_motor_with_TIMER1_fast_pwm_ICRA_wave_B(90);
			_delay_ms(1000);
			Servo_motor_with_TIMER1_fast_pwm_ICRA_wave_B(0);
			LCD_clr_screen();
			LCD_send_string("there are ");
			LCD_send_data(count+48);
			LCD_send_string(" cars");
			LCD_move_cursor(2,1);
			LCD_send_string("garage has space");
			flag_2=1;
		}else if (0==IR_2)
		{
			flag_2=0;
		}
    }
}

