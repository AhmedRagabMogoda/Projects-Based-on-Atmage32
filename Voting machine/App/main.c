/*
 * voting_machine.c
 *
 * Created: 7/27/2024 11:30:06 PM
 * Author : Ahmed Ragab
 */ 
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include "LCD.h"
#include "BUTTON.h"
#include "LED.h"



int main(void)
{
   LCD_init();
   BUTTON_init_with_pull_up('B',0);
   BUTTON_init_with_pull_up('B',1);
   BUTTON_init_with_pull_up('B',2);
   BUTTON_init_with_pull_up('B',3);
   BUTTON_init_with_pull_up('B',4);
   BUTTON_init('A',0);
   BUTTON_init('A',1);
   LED_init('C',0);
   LED_init('C',1);
   LED_init('C',2);
   LED_init('C',3);
   LED_init('C',4);
   LED_init('C',5);
   LED_init('C',6);
   char counter1=0,counter2=0,counter3=0,counter4=0;
   char value1,value2,value3,value4;
   char flag1,flag2,flag3,flag4;
   char enter=0;
   char responsible_flag=0;
   LCD_send_string("A=0");
   LCD_move_cursor(1,12);
   LCD_send_string("B=0");
   LCD_move_cursor(2,1);
   LCD_send_string("C=0");
   LCD_move_cursor(2,12);
   LCD_send_string("D=0");
    while (1) 
    {
		flag1=flag2=flag3=flag4=0;
		value1=counter1;
		value2=counter2;
		value3=counter3;
		value4=counter4;
		do 
		{
			LED_turn_off('C',5);
			LED_turn_on('C',4);
			if (1==BUTTON_read('A',1))
			{
				LED_turn_off('C',6);
				counter1=counter2=counter3=counter4=0;
				LCD_clr_screen();
				LCD_move_cursor(1,1);
				LCD_send_string("A=0");
				LCD_move_cursor(1,12);
				LCD_send_string("B=0");
				LCD_move_cursor(2,1);
				LCD_send_string("C=0");
				LCD_move_cursor(2,12);
				LCD_send_string("D=0");
			}
			responsible_flag=BUTTON_read('A',0);
			if (1==responsible_flag)
			{
				LED_turn_off('C',6);
				LED_turn_off('C',4);
				LED_turn_on('C',5);
			}
		} while (0==responsible_flag);
		
label:	do
		{
			if (0==BUTTON_read('B',0))
			{
				if (0==flag1)
				{
					if (1==flag2)
					{
						counter2--;
						flag2=0;
						LED_turn_off('C',1);
					}
					else if (1==flag3)
					{
						counter3--;
						flag3=0;
						LED_turn_off('C',2);
					}
					else if (1==flag4)
					{
						counter4--;
						flag4=0;
						LED_turn_off('C',3);
					}
					else
					{
						
					}
					counter1++;
					flag1=1;
					LED_turn_on('C',0);
				}
				else
				{
					counter1--;
					flag1=0;
					LED_turn_off('C',0);
				}
				
			}
			else if (0==BUTTON_read('B',1))
			{
				if (0==flag2)
				{
					if (1==flag1)
					{
						counter1--;
						flag1=0;
						LED_turn_off('C',0);
					}
					else if (1==flag3)
					{
						counter3--;
						flag3=0;
						LED_turn_off('C',2);
					}
					else if (1==flag4)
					{
						counter4--;
						flag4=0;
						LED_turn_off('C',3);
					}
					counter2++;
					flag2=1;
					LED_turn_on('C',1);
				}
				else
				{
					counter2--;
					flag2=0;
					LED_turn_off('C',1);
				}
				
			}
			else if (0==BUTTON_read('B',2))
			{
				if (0==flag3)
				{
					if (1==flag1)
					{
						counter1--;
						flag1=0;
						LED_turn_off('C',0);
					}
					else if (1==flag2)
					{
						counter2--;
						flag2=0;
						LED_turn_off('C',1);
					}
					else if (1==flag4)
					{
						counter4--;
						flag4=0;
						LED_turn_off('C',3);
					}
					counter3++;
					flag3=1;
					LED_turn_on('C',2);
				}
				else
				{
					counter3--;
					flag3=0;
					LED_turn_off('C',2);
				}
				
			}
			else if (0==BUTTON_read('B',3))
			{
				if (0==flag4)
				{
					if (1==flag1)
					{
						counter1--;
						flag1=0;
						LED_turn_off('C',0);
					}
					else if (1==flag2)
					{
						counter2--;
						flag2=0;
						LED_turn_off('C',1);
					}
					else if (1==flag3)
					{
						counter3--;
						flag3=0;
						LED_turn_off('C',2);
					}
					counter4++;
					flag4=1;
					LED_turn_on('C',3);
				}
				else
				{
					counter4--;
					flag4=0;
					LED_turn_off('C',3);
				}
				
			}
			_delay_ms(200);
			enter=BUTTON_read('B',4);
			if (enter==0 && value1==counter1 && value2==counter2 && value3==counter3 && value4==counter4)
			{
				LCD_clr_screen();
				LCD_send_string("please vote");
				_delay_ms(1500);
				LCD_clr_screen();
				LCD_move_cursor(1,1);
				LCD_send_string("A=");
				LCD_send_data((counter1/10)+48);
				LCD_send_data((counter1%10)+48);
				LCD_move_cursor(1,12);
				LCD_send_string("B=");
				LCD_send_data((counter2/10)+48);
				LCD_send_data((counter2%10)+48);
				LCD_move_cursor(2,1);
				LCD_send_string("C=");
				LCD_send_data((counter3/10)+48);
				LCD_send_data((counter3%10)+48);
				LCD_move_cursor(2,12);
				LCD_send_string("D=");
				LCD_send_data((counter4/10)+48);
				LCD_send_data((counter4%10)+48);
				goto label;
			}
			if (0==enter)
			{
				LED_turn_off('C',0);
				LED_turn_off('C',1);
				LED_turn_off('C',2);
				LED_turn_off('C',3);
				LED_turn_on('C',6);
			}
			
		} while (1==enter);
		
		LCD_move_cursor(1,3);
		LCD_send_data((counter1/10)+48);
		LCD_send_data((counter1%10)+48);
		LCD_move_cursor(1,14);
		LCD_send_data((counter2/10)+48);
		LCD_send_data((counter2%10)+48);
		LCD_move_cursor(2,3);
		LCD_send_data((counter3/10)+48);
		LCD_send_data((counter3%10)+48);
		LCD_move_cursor(2,14);
		LCD_send_data((counter4/10)+48);
		LCD_send_data((counter4%10)+48);;
		
    }
}
