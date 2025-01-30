/*
 * Calculator.c
 *
 * Created: 8/4/2024 9:21:58 PM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "LCD.h"
#include "KEYBAD.h"


int main(void)
{
	LCD_init();
	KEYBAD_init();
	long int operand_1,operand_2;
	char operation;
	long int result;
	char operand_1_str[5];
	char operand_2_str[5];
	char key;
	char counter_1,counter_2;
	
   
    while (1) 
    {
label_1:
		counter_1=counter_2=0;
		operand_1_str[0]=operand_1_str[1]=operand_1_str[2]=operand_1_str[3]=0;
		operand_2_str[0]=operand_2_str[1]=operand_2_str[2]=operand_2_str[3]=0;
		operation='0';
		LCD_clr_screen();
		LCD_send_cmd(CURSOR_ON_DISPLAY_ON);
		do   //Enter first number
		{
			key=KEYBAD_check_press();
			if ( (('+' == key || '-' == key ) && 0==operand_1_str[0]) || '.'==key || '0'== key || '1'== key || '2'== key || '3'== key || '4'== key || '5'== key || '6'== key || '7'== key || '8'== key || '9'== key)
			{
				operand_1_str[counter_1]=key;
				LCD_send_data(operand_1_str[counter_1]);
				counter_1++;
				_delay_ms(250);
			}
			else if ('+'==key || '-'==key || '*'==key || '/'==key || '^'==key || 'S'==key || '%'==key)
			{
				operation=key;
				if ('S'==operation)
				{
					LCD_clr_screen();
					LCD_send_string("Sqrt(");
				}
				else 
				{
					LCD_send_data(operation);
				}
				break;	
			}
			else if('A'== key)
			{
				goto label_1;
			}
		} while (4!=counter_1);
		_delay_ms(200);
		operand_1_str[counter_1] = '\0';
		operand_1=atoi(operand_1_str);
		
		if ('0'==operation)
		{
			do    //Enter operation
			{
				key=KEYBAD_check_press();
				if ('+'==key || '-'==key || '*'==key || '/'==key || '^'==key || 'S'==key || '%'==key)
				{
					operation=key;
					if ('S'==operation)
					{
						LCD_clr_screen();
						LCD_send_string("Sqrt(");
					}
					else
					{
						LCD_send_data(operation);
					}
					break;
				}
				else if('A'== key)
				{
					goto label_1;
				}
			} while (1);
			_delay_ms(250);
		}
label_2:if ((('*'==key || '/'==key || '^'==key || '%'==key) && 0==operand_1) || ('S'==key && 0!=operand_1))
		{
			LCD_clr_screen();
			LCD_send_string("Erorr");
			_delay_ms(1000);
			goto label_1;
		}
		
		do   //Enter second number
		{
			key=KEYBAD_check_press();
			if ((('+' == key || '-' == key ) && 0==operand_2_str[0]) || '.'==key || '0'== key ||'1'== key || '2'== key || '3'== key || '4'== key || '5'== key || '6'== key || '7'== key || '8'== key || '9'== key)
			{
				operand_2_str[counter_2]=key;
				LCD_send_data(operand_2_str[counter_2]);
				counter_2++;
				_delay_ms(250);
			}
			else if('A'== key)
			{
				goto label_1;
			}
			else if('='== key)
			{
				break;
			}
		} while (4!=counter_2);
		operand_2_str[counter_2] = '\0';
		operand_2=atoi(operand_2_str);
		
		do    // Calculate result
		{
			key=KEYBAD_check_press();
			if ('='==key)
			{
				if ('S'==operation)
				{
					LCD_send_data(')');
				}
				LCD_send_data('=');
				switch(operation)
				{
					case '+':
					{
						result=operand_1 + operand_2;
						break;
					}
					case '-':
					{
						result=operand_1 - operand_2;
						break;
					}
					case '*':
					{
						result=operand_1 * operand_2;
						break;
					}
					case '/':
					{
						if (operand_2!=0)
						{
							result=operand_1 / operand_2;
							break;
						}
						else
						{
							LCD_clr_screen();
							LCD_send_string("Error: Division ");
							LCD_move_cursor(2,4);
							LCD_send_string("by zero");
							continue;
						}
					}
					case '^':
					{
						result=(pow(operand_1,operand_2)+1);
						break;
					}
					case 'S':
					{
						if (operand_2>=0)
						{
							result=sqrt(operand_2);
						}
						else
						{
							LCD_clr_screen();
							LCD_send_string("Error: Negative");
							LCD_move_cursor(2,5);
							LCD_send_string("sqrt");
							continue;
						}
						break;
					}
					case '%':
					{
						result=operand_1 % operand_2;
						break;
					}
					default:
					{
						break;
					}
				}
				
				 //display result can handle negative and positive number
				char result_str[10]; 
				sprintf(result_str, "%ld", result);
				//LCD_move_cursor(2,8);
				LCD_send_string(result_str);
				
				 /*
				 // other away to diplay result but can't handle negative number
				if (result<10)
				{
					LCD_send_data(result+48);
				}
				else if (result<100)
				{
					LCD_send_data(result/10+48);
					LCD_send_data(result%10+48);
				}
				else if (result<1000)
				{
					LCD_send_data(result/100+48);
					LCD_send_data((result%100)/10+48);
					LCD_send_data(result%10+48);
				}
				else if (result<10000)
				{
					LCD_send_data(result/1000+48);
					LCD_send_data((result%1000)/100+48);
					LCD_send_data((result%100)/10+48);
					LCD_send_data(result%10+48);
				}
				else if (result<100000)
				{
					LCD_send_data(result/10000+48);
					LCD_send_data((result%10000)/1000+48);
					LCD_send_data((result%1000)/100+48);
					LCD_send_data((result%100)/10+48);
					LCD_send_data(result%10+48);
			
				}
				else if (result<1000000)
				{
				LCD_send_data(result/100000+48);
				LCD_send_data((result%100000)/10000+48);
				LCD_send_data((result%10000)/1000+48);
				LCD_send_data((result%1000)/100+48);
				LCD_send_data((result%100)/10+48);
				LCD_send_data(result%10+48);
				}
				else if (result<10000000)
				{
					LCD_move_cursor(2,10);
					LCD_send_data(result/1000000+48);
					LCD_send_data((result%1000000)/100000+48);
					LCD_send_data((result%100000)/10000+48);
					LCD_send_data((result%10000)/1000+48);
					LCD_send_data((result%1000)/100+48);
					LCD_send_data((result%100)/10+48);
					LCD_send_data(result%10+48);
				}
				else if (result<100000000)
				{
					LCD_move_cursor(2,9);
					LCD_send_data(result/10000000+48);
					LCD_send_data((result%10000000)/1000000+48);
					LCD_send_data((result%1000000)/100000+48);
					LCD_send_data((result%100000)/10000+48);
					LCD_send_data((result%10000)/1000+48);
					LCD_send_data((result%1000)/100+48);
					LCD_send_data((result%100)/10+48);
					LCD_send_data(result%10+48);
				}
				*/
			}
		} while ('='!=key);
		LCD_send_cmd(CURSOR_Off_DISPLAY_ON);
		_delay_ms(250);
		
		do    // Continue with result
		{
			key=KEYBAD_check_press();
			if ('+'==key || '-'==key || '*'==key || '/'==key || '^'==key || 'S'==key || '%'==key)
			{
				LCD_clr_screen();
				LCD_send_string("Ans");
				operation = key;
				operand_1 = result;
				counter_2 = 0;
				operand_2_str[0] = 0;
				operand_2_str[1] = 0;
				operand_2_str[2] = 0;
				operand_2_str[3] = 0;
				if ('S' == key)
				{
					LCD_send_string("Sqrt(");
				}
				else
				{
					LCD_send_data(operation);
				}
				_delay_ms(250);
				goto label_2;
			}
		} while (NOT_pressed==key || '='==key);		
    }
}