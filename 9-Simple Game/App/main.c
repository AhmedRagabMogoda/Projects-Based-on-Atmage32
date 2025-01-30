/*
 * Simple_Game.c
 *
 * Created: 8/20/2024 12:01:40 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include "LCD.h"
#include "KEYBAD.h"
#include "TIMER.h"
#define F_CPU 8000000UL
#include <util/delay.h>

volatile int flag1,flag2;
int main(void)
{
	LCD_init();
	KEYBAD_init();
	TIMER0_CTC_init_with_interrupt();
	char ret_key;
	int num1,num2,num3;
	int count1=0,count2=0,count3=0;
	char num1_str[4];
	char num2_str[4];
	char num3_str[4];
	
	while(1)
	{
		flag2=0;
		flag1=0;
		LCD_clr_screen();
		LCD_send_string("8+?? =13");
		do
		{
			ret_key=KEYBAD_check_press();
			if ('='==ret_key || '0'==ret_key || '1'==ret_key|| '2'==ret_key || '3'==ret_key || '4'==ret_key || '5'==ret_key || '6'==ret_key || '7'==ret_key || '8'==ret_key || '9'==ret_key)
			{
				if ('='==ret_key)
				{
					break;
				}
				else
				{
					if (0==count1)
					{
						LCD_move_cursor(1,3);
						LCD_send_string("  ");
						LCD_move_cursor(1,3);
					}
					LCD_send_data(ret_key);
					num1_str[count1]=ret_key;
					count1++;
					_delay_ms(250);
				}
			}
			if (flag1>=1000)
			{
				flag2=1;
			}
		} while (0==flag2);
		_delay_ms(200);
		num1_str[count1]='\0';
		num1=atoi(num1_str);
		if (NOT_pressed==ret_key)
		{
			LCD_clr_screen();
			LCD_send_string("   Time over");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:0/10");
			break;
		}
		else if (5==num1)
		{
			LCD_clr_screen();
			LCD_send_string("   Well Done");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:2/10");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string("   Next Level");
			_delay_ms(1000);
		}
		else
		{
			LCD_clr_screen();
			LCD_send_string("   Wrong Answer");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:0/10");
			break;
		}
		LCD_clr_screen();
		LCD_send_string("68+?? =100");
		flag1=0;
		do
		{
			ret_key=KEYBAD_check_press();
			if ('='==ret_key || '0'==ret_key || '1'==ret_key|| '2'==ret_key || '3'==ret_key || '4'==ret_key || '5'==ret_key || '6'==ret_key || '7'==ret_key || '8'==ret_key || '9'==ret_key)
			{
				if ('='==ret_key)
				{
					break;
				}
				else
				{
					if (0==count2)
					{
						LCD_move_cursor(1,4);
						LCD_send_string("  ");
						LCD_move_cursor(1,4);
					}
					LCD_send_data(ret_key);
					num2_str[count2]=ret_key;
					count2++;
					_delay_ms(250);
				}
			}
			if (flag1>=1000)
			{
				flag2=1;
			}
		} while (0==flag2);
		_delay_ms(200);
		num2_str[count2]='\0';
		num2=atoi(num2_str);
		if (NOT_pressed==ret_key)
		{
			LCD_clr_screen();
			LCD_send_string("   Time over");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:2/10");
			break;
		}
		else if (32==num2)
		{
			LCD_clr_screen();
			LCD_send_string("   Well Done");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:6/10");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string("   Next Level");
			_delay_ms(1000);
		}
		else
		{
			LCD_clr_screen();
			LCD_send_string("   Wrong Answer");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:2/10");
			break;
			
		}
		LCD_clr_screen();
		LCD_send_string("150+?? =322");
		flag1=0;
		do
		{
			ret_key=KEYBAD_check_press();
			if ('='==ret_key || '0'==ret_key || '1'==ret_key|| '2'==ret_key || '3'==ret_key || '4'==ret_key || '5'==ret_key || '6'==ret_key || '7'==ret_key || '8'==ret_key || '9'==ret_key)
			{
				if ('='==ret_key)
				{
					break;
				}
				else
				{
					if (0==count3)
					{
						LCD_move_cursor(1,5);
						LCD_send_string("  ");
						LCD_move_cursor(1,5);
					}
					LCD_send_data(ret_key);
					num3_str[count3]=ret_key;
					count3++;
					_delay_ms(250);
				}
			}
			if (flag1>=1000)
			{
				flag2=1;
			}
		} while (0==flag2);
		_delay_ms(200);
		num3_str[count3]='\0';
		num3=atoi(num3_str);
		if (NOT_pressed==ret_key)
		{
			LCD_clr_screen();
			LCD_send_string("   Time over");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:6/10");
			break;
		}
		else if (172==num3)
		{
			LCD_clr_screen();
			LCD_send_string("   Well Done");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:10/10");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string(" Congratulation");
			break;
			
		}
		else
		{
			LCD_clr_screen();
			LCD_send_string("   Wrong Answer");
			LCD_move_cursor(2,4);
			LCD_send_string("Score:6/10");
			break;
			
		}
	}
	/* //note we not used goto in embedded system
	do 
	{
		ret_key=KEYBAD_check_press();
		if ('A'==ret_key)
		{
			count1=count2=count3=0;
			num1_str[0]=num1_str[1]=num1_str[2]=0;
			num2_str[0]=num2_str[1]=num2_str[2]=0;
			num3_str[0]=num3_str[1]=num3_str[2]=0;
			
			goto label;  //write label: in line 30
		}
	} while('A'!=ret_key);
	*/
}

ISR(TIMER0_COMP_vect)
{
	flag1++;
}

