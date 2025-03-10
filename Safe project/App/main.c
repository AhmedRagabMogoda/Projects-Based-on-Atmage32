/*
 * safe_project.c
 *
 * Created: 7/28/2024 5:30:54 PM
 * Author : Ahmed Ragab
 */ 
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include "EEPROM.h"
#include "LCD.h"
#include "KEYBAD.h"
#include "DC_MOTOR.h"

void check_password(void);
void set_password(void);

#define EEPROM_default_value 0xff
#define EEPROM_status_address 0x20
#define EEPROM_first_address_of_password 0x21
#define EEPROM_scond_address_of_password 0x22
#define EEPROM_thrid_address_of_password 0x23
#define EEPROM_fourth_address_of_password 0x24
#define max_tries 3

char password_array[4];

int main(void)
{
	LCD_init();
	KEYPAD_init();
	DC_motor_init();
	char counter,value;
	char tries=max_tries;
	
	if (EEPROM_default_value==EEPROM_read(EEPROM_status_address))
	{
		set_password();
	}
    
	
    while (1) 
    {
label_0:check_password();
		if (password_array[0]==EEPROM_read(EEPROM_first_address_of_password) && password_array[1]==EEPROM_read(EEPROM_scond_address_of_password) && password_array[2]==EEPROM_read(EEPROM_thrid_address_of_password) && password_array[3]==EEPROM_read(EEPROM_fourth_address_of_password))
		{
			LCD_clr_screen();
			LCD_send_string("correct password");
			LCD_move_cursor(2,1);
			LCD_send_string("the safe opened");
			DC_motor_Forward();
			_delay_ms(1700);
			DC_motor_Stop();
label_1:	do 
			{
				value=KEYPAD_read();
			} while ('A'!=value);
label_2:	LCD_clr_screen();
			LCD_send_string("1-change pass");
			LCD_move_cursor(2,1);
			LCD_send_string("2-Back");
			do 
			{
				value=KEYPAD_read();
			} while (value!='1' && value!='2');
			if ('1'==value)
			{
				_delay_ms(200);
				check_password();
				if (password_array[0]==EEPROM_read(EEPROM_first_address_of_password) && password_array[1]==EEPROM_read(EEPROM_scond_address_of_password) && password_array[2]==EEPROM_read(EEPROM_thrid_address_of_password) && password_array[3]==EEPROM_read(EEPROM_fourth_address_of_password))
				{
					set_password();
					LCD_clr_screen();
					LCD_send_string("password changed");
					LCD_move_cursor(2,1);
					LCD_send_string("  successfully");
					_delay_ms(2000);
					LCD_clr_screen();
					LCD_send_string("the safe opened");
					goto label_1;
				}
				else
				{
					LCD_clr_screen();
					LCD_send_string("wrong password");
					_delay_ms(2000);
					LCD_clr_screen();
					LCD_send_string("please try again");
					_delay_ms(2000);
					goto label_2;
					
				}
			}
			else //value=2
			{
				LCD_clr_screen();
				LCD_send_string("the safe opened");
				goto label_1;
			}
		}
		else
		{
			tries-=1;
			if (tries>0)
			{
				LCD_clr_screen();
				LCD_send_string("wrong password");
				_delay_ms(2000);
				LCD_clr_screen();
				LCD_send_string("please try again");
				LCD_move_cursor(2,1);
				LCD_send_string("tries left:");
				LCD_send_data(tries+48);
				_delay_ms(2000);
				
			}
			else
			{
				LCD_clr_screen();
				LCD_send_string("wrong password");
				_delay_ms(1000);
				LCD_clr_screen();
				LCD_send_string("the safe closed");
				_delay_ms(2000);
				LCD_clr_screen();
				LCD_send_string("please try again");
				LCD_move_cursor(2,1);
				LCD_send_string("in 60 minutes:");
				for (counter=0;counter<60;counter++)
				{
					_delay_ms(100);
					LCD_move_cursor(2,15);
					LCD_send_data(counter/10+48);
					LCD_send_data(counter%10+48);
				}
				tries=max_tries;
				goto label_0;
				
			}
		}
		
    }
	
}

void set_password(void)
{
	char counter1,ret_val;
	LCD_clr_screen();
	LCD_send_string("set pass:");
	for (counter1=0;counter1<4;counter1++)
	{
		do
		{
			ret_val=KEYPAD_read();
		} while (0xff==ret_val);
		LCD_send_data(ret_val);
		_delay_ms(500);
		LCD_move_cursor(1,10+counter1);
		LCD_send_data('*');
		EEPROM_write(EEPROM_first_address_of_password+counter1,ret_val);
	}
	EEPROM_write(EEPROM_status_address,0x00);
}

void check_password(void)
{
	char counter1,ret_val;
	LCD_clr_screen();
	LCD_send_string("check pass:");
	for (counter1=0;counter1<4;counter1++)
	{
		do
		{
			ret_val=KEYPAD_read();
		} while (0xff==ret_val);
		LCD_send_data(ret_val);
		_delay_ms(500);
		LCD_move_cursor(1,12+counter1);
		LCD_send_data('*');
		password_array[counter1]=ret_val;
	}
}