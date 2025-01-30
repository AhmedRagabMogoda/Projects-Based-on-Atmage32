/*
 * Smart_Home_master.c
 *
 * Created: 9/5/2024 12:14:24 AM
 * Author : Ahmed Ragab
 */ 

#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "EEPROM.h"
#include "LCD.h"
#include "LED.h"
#include "KEYBAD.h"
#include "TIMER.h"
#include "SPI.h"
#include "main_config.h"
unsigned char KEYBAD_user(void);
unsigned char led_R1,led_R2,led_R3,led_R4;
unsigned char fan_R1,fan_R2,fan_R3,fan_R4;
unsigned char tv,air_cond,door,temperature;
volatile unsigned long time_out;
int main(void)
{
	LCD_init();
	KEYBAD_init();
	SPI_master_init();
	TIMER0_CTC_init_with_interrupt();
	LED_init(ADMIN_PORT,ADMIN_PIN);
	LED_init(GUEST_PORT,GUEST_PIN);
	signed char counter;
	unsigned char key_value;
	unsigned char value_status;
	unsigned char mode;
	unsigned char menu;
	unsigned char led_status,fan_status,room_status;
	unsigned char device;
	unsigned char tries=TRIES_ALLOWED;
	unsigned char pass_status;
	unsigned char pass[PASS_SIZE];
	unsigned char check_pass[PASS_SIZE];
	
	LCD_move_cursor(2,1);
	LCD_send_string("   Welcome to Smart");
	LCD_move_cursor(3,1);
	LCD_send_string("     Home system");
	_delay_ms(1000);
	value_status=EEPROM_read(ADMIN_PASS_STATUS_ADDRESS);
	if (PASS_NOT_SET==value_status)
	{
		LCD_clr_screen();
		LCD_move_cursor(2,1);
		LCD_send_string("     login for");
		LCD_move_cursor(3,1);
		LCD_send_string("     first time");
		_delay_ms(1000);
		LCD_clr_screen();
		LCD_send_string(" set Admin password");
		LCD_move_cursor(3,1);
		LCD_send_string("password:");
		for (counter=0;counter<PASS_SIZE;counter++)
		{
			do 
			{
				key_value=KEYBAD_check_press();
			} while (NOT_pressed==key_value);
			LCD_send_data(key_value);
			_delay_ms(500);
			LCD_move_cursor(3,10+counter);
			LCD_send_data(PASSWORD_SYMBOL);
			EEPROM_write(EEPROM_ADMIN_ADDRESS+counter,key_value);
			key_value=NOT_pressed;
		}
		_delay_ms(500);
		LCD_clr_screen();
		LCD_move_cursor(2,1);
		LCD_send_string("   password save");
		_delay_ms(500);
		EEPROM_write(ADMIN_PASS_STATUS_ADDRESS,PASS_SET);
	}
	
	led_R1=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	led_R1=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	led_R2=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	led_R3=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	led_R4=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	fan_R1=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	fan_R2=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	fan_R3=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	fan_R4=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	door=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	tv=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	air_cond=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	temperature=SPI_master_transmit_and_receive_data(DUMMY_DATA);
	_delay_ms(50);
	
	while (1) 
    {
		tries=EEPROM_read(ADDRESS_TRIES);
		if (0==tries)
		{
			LCD_clr_screen();
			LCD_send_string("  the system closed");
			LCD_move_cursor(2,1);
			LCD_send_string("  please try again");
			LCD_move_cursor(3,1);
			LCD_send_string("     after:");
			for (counter=EEPROM_read(ADDRESS_TRIES_count);counter>=0;counter--)
			{
				LCD_move_cursor(3,12);
				LCD_send_data(counter/10+ASCII_ZERO);
				LCD_send_data(counter%10+ASCII_ZERO);
				EEPROM_write(ADDRESS_TRIES_count,counter);
				_delay_ms(1000);
			}
			tries=TRIES_ALLOWED;
			EEPROM_write(ADDRESS_TRIES,TRIES_ALLOWED);
			EEPROM_write(ADDRESS_TRIES_count,TIME_CLOSED);
		}
		
		
		LCD_clr_screen();
		LCD_send_string("   select mode:");
		LCD_move_cursor(3,1);
		LCD_send_string("1-Admin     2-Guest");
		do
		{
			SPI_master_transmit_and_receive_data(DUMMY_DATA);
			_delay_ms(20);
			key_value=KEYBAD_check_press();
		} while (NOT_pressed==key_value);
		mode=key_value;
		switch(mode)
		{
			case ADMIN :
			{
				while(tries>0)
				{
					LCD_clr_screen();
					LCD_send_string("     Admin mode");
					LCD_move_cursor(3,1);
					LCD_send_string("check pass:");
					for (counter=0;counter<PASS_SIZE;counter++)
					{
						key_value=KEYBAD_user();
						LCD_send_data(key_value);
						_delay_ms(500);
						LCD_move_cursor(3,12+counter);
						LCD_send_data(PASSWORD_SYMBOL);
						pass[counter]=key_value;
						check_pass[counter]=EEPROM_read(EEPROM_ADMIN_ADDRESS+counter);
						key_value=NOT_pressed;
					}
					
					for (counter=0;counter<PASS_SIZE;counter++)
					{
						if (check_pass[counter] != pass[counter])
						{
							pass_status=FALSE;
							break;
						}
						pass_status=TRUE;
					}
					
					if (pass_status == FALSE)
					{
						tries-=1;
						EEPROM_write(ADDRESS_TRIES,tries);
						EEPROM_write(ADDRESS_TRIES_count,TIME_CLOSED);
						LCD_clr_screen();
						LCD_move_cursor(2,1);
						LCD_send_string("   Wrong password");
						LCD_move_cursor(3,1);
						LCD_send_string("   Tries left:");
						LCD_send_data(tries+ASCII_ZERO);
						_delay_ms(1000);
						break;
					}
					else
					{
						tries=TRIES_ALLOWED;
						EEPROM_write(ADDRESS_TRIES,tries);
						LCD_clr_screen();
						LCD_move_cursor(2,1);
						LCD_send_string("   Right password");
						_delay_ms(500);
						LED_turn_on(ADMIN_PORT,ADMIN_PIN);
						while(time_out<TIME_MAX)
						{
							LCD_clr_screen();
							LCD_send_string("Control List:");
							LCD_move_cursor(2,1);
							LCD_send_string(" 1-Room1   2-Room2");
							LCD_move_cursor(3,1);
							LCD_send_string(" 3-Room1   4-Room4");
							LCD_move_cursor(4,1);
							LCD_send_string(" 0-back    A-more");
							menu=KEYBAD_user();
							if (ROOM1_MENU==menu)
							{
								while(time_out<TIME_MAX)
								{
									LCD_clr_screen();
									LCD_send_string("      Room one");
									LCD_move_cursor(3,1);
									LCD_send_string("0-back  1-led  2-fan");
									device=KEYBAD_user();
									if (LED==device)
									{
										while (1)
										{
											LCD_clr_screen();
											LCD_send_string("      Room one");
											LCD_move_cursor(2,1);
											LCD_send_string("led:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (led_R1==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											led_status=KEYBAD_user();
											if (ON_STATUS==led_status)
											{
												led_R1=ON;
												SPI_master_transmit_and_receive_data(ROOM1_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==led_status)
											{
												led_R1=OFF;
												SPI_master_transmit_and_receive_data(ROOM1_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==led_status)
											{
												break;
											}
										}
									}
									else if (FAN==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room one");
											LCD_move_cursor(2,1);
											LCD_send_string("fan:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (fan_R1==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											fan_status=KEYBAD_user();
											if (ON_STATUS==fan_status)
											{
												fan_R1=ON;
												SPI_master_transmit_and_receive_data(ROOM1_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==fan_status)
											{
												fan_R1=OFF;
												SPI_master_transmit_and_receive_data(ROOM1_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==fan_status)
											{
												break;
											}
										}
									}
									else if (RET_STATUS==device)
									{
										break;
									}
								}
							}
							else if (ROOM2_MENU==menu)
							{
								while(time_out<TIME_MAX)
								{
									LCD_clr_screen();
									LCD_send_string("      Room two");
									LCD_move_cursor(3,1);
									LCD_send_string("0-back  1-led  2-fan");
									device=KEYBAD_user();
									if (LED==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room two");
											LCD_move_cursor(2,1);
											LCD_send_string("led:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (led_R2==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											led_status=KEYBAD_user();
											if (ON_STATUS==led_status)
											{
												led_R2=ON;
												SPI_master_transmit_and_receive_data(ROOM2_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==led_status)
											{
												led_R2=OFF;
												SPI_master_transmit_and_receive_data(ROOM2_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==led_status)
											{
												break;
											}
										}
									}
									else if (FAN==device)
									{
										while (1)
										{
											LCD_clr_screen();
											LCD_send_string("      Room two");
											LCD_move_cursor(2,1);
											LCD_send_string("fan:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (fan_R2==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											fan_status=KEYBAD_user();
											if (ON_STATUS==fan_status)
											{
												fan_R2=ON;
												SPI_master_transmit_and_receive_data(ROOM2_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==fan_status)
											{
												fan_R2=OFF;
												SPI_master_transmit_and_receive_data(ROOM2_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==fan_status)
											{
												break;
											}
										}
									}
									else if (RET_STATUS==device)
									{
										break;
									}
								}
							}
							else if (ROOM3_MENU==menu)
							{
								while(time_out<TIME_MAX)
								{
									LCD_clr_screen();
									LCD_send_string("      Room three");
									LCD_move_cursor(3,1);
									LCD_send_string("0-back  1-led  2-fan");
									device=KEYBAD_user();
									if (LED==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room three");
											LCD_move_cursor(2,1);
											LCD_send_string("led:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (led_R3==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											led_status=KEYBAD_user();
											if (ON_STATUS==led_status)
											{
												led_R3=ON;
												SPI_master_transmit_and_receive_data(ROOM3_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==led_status)
											{
												led_R3=OFF;
												SPI_master_transmit_and_receive_data(ROOM3_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==led_status)
											{
												break;
											}
										}
									}
									else if (FAN==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room three");
											LCD_move_cursor(2,1);
											LCD_send_string("fan:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (fan_R3==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											fan_status=KEYBAD_user();
											if (ON_STATUS==fan_status)
											{
												fan_R3=ON;
												SPI_master_transmit_and_receive_data(ROOM3_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==fan_status)
											{
												fan_R3=OFF;
												SPI_master_transmit_and_receive_data(ROOM3_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==fan_status)
											{
												break;
											}
										}
									}
									else if (RET_STATUS==device)
									{
										break;
									}
								}
							}
							else if (ROOM4_MENU==menu)
							{
								while(time_out<TIME_MAX)
								{
									LCD_clr_screen();
									LCD_send_string("      Room four");
									LCD_move_cursor(3,1);
									LCD_send_string("0-back  1-led  2-fan");
									device=KEYBAD_user();
									if (LED==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room four");
											LCD_move_cursor(2,1);
											LCD_send_string("led:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (led_R4==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											led_status=KEYBAD_user();
											if (ON_STATUS==led_status)
											{
												led_R4=ON;
												SPI_master_transmit_and_receive_data(ROOM4_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==led_status)
											{
												led_R4=OFF;
												SPI_master_transmit_and_receive_data(ROOM4_MENU);
												_delay_ms(50);_delay_ms(50);
												SPI_master_transmit_and_receive_data(LED);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==led_status)
											{
												break;
											}
										}
									}
									else if (FAN==device)
									{
										while (time_out<TIME_MAX)
										{
											LCD_clr_screen();
											LCD_send_string("      Room four");
											LCD_move_cursor(2,1);
											LCD_send_string("fan:");
											LCD_move_cursor(3,1);
											LCD_send_string("0-back 1-on 2-off");
											if (fan_R4==ON)
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,5);
												LCD_send_string("   ");
												LCD_move_cursor(2,5);
												LCD_send_string("off");
											}
											fan_status=KEYBAD_user();
											if (ON_STATUS==fan_status)
											{
												fan_R4=ON;
												SPI_master_transmit_and_receive_data(ROOM4_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==fan_status)
											{
												fan_R4=OFF;
												SPI_master_transmit_and_receive_data(ROOM4_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(FAN);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==fan_status)
											{
												break;
											}
										}
									}
									else if (RET_STATUS==device)
									{
										break;
									}
								}
							}
							else if (MORE_MENU==menu)
							{
								while(time_out<TIME_MAX)
								{
									LCD_clr_screen();
									LCD_send_string(" 5-Home Door  6-TV ");
									LCD_move_cursor(2,1);
									LCD_send_string(" 7-air_conditioning");
									LCD_move_cursor(3,1);
									LCD_send_string(" 8-change password");
									LCD_move_cursor(4,1);
									LCD_send_string(" 9-Temp      0-back");
									menu=KEYBAD_user();
									if (HOME_DOOR==menu)
									{
										LCD_clr_screen();
										LCD_send_string("   Home Door");
										LCD_move_cursor(2,1);
										LCD_send_string("status:");
										LCD_move_cursor(3,1);
										LCD_send_string("0-back 1-on 2-closed");
										while(time_out<TIME_MAX)
										{
											if (door==ON)
											{
												LCD_move_cursor(2,8);
												LCD_send_string("         ");
												LCD_move_cursor(2,8);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,8);
												LCD_send_string("        ");
												LCD_move_cursor(2,8);
												LCD_send_string("closed");
											}
											room_status=KEYBAD_user();
											if (ON_STATUS==room_status)
											{
												door=ON;
												SPI_master_transmit_and_receive_data(HOME_DOOR);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==room_status)
											{
												door=OFF;
												SPI_master_transmit_and_receive_data(HOME_DOOR);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==room_status)
											{
												break;
											}
										}
									}
									else if (AIRCONDITIONING_MENU==menu)
									{
										LCD_clr_screen();
										LCD_send_string("Central coditioning");
										LCD_move_cursor(2,1);
										LCD_send_string("status:");
										LCD_move_cursor(3,1);
										LCD_send_string("0-back 1-on 2-off");
										while(time_out<TIME_MAX)
										{
											if (air_cond==ON)
											{
												LCD_move_cursor(2,8);
												LCD_send_string("   ");
												LCD_move_cursor(2,8);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,8);
												LCD_send_string("   ");
												LCD_move_cursor(2,8);
												LCD_send_string("off");
											}
											room_status=KEYBAD_user();
											if (ON_STATUS==room_status)
											{
												air_cond=ON;
												SPI_master_transmit_and_receive_data(AIRCONDITIONING_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==room_status)
											{
												air_cond=OFF;
												SPI_master_transmit_and_receive_data(AIRCONDITIONING_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==room_status)
											{
												break;
											}
										}
									}
									else if (TV_MENU==menu)
									{
										LCD_clr_screen();
										LCD_send_string("    Television");
										LCD_move_cursor(2,1);
										LCD_send_string("status:");
										LCD_move_cursor(3,1);
										LCD_send_string("0-back 1-on 2-off");
										while(time_out<TIME_MAX)
										{
											if (tv==ON)
											{
												LCD_move_cursor(2,8);
												LCD_send_string("   ");
												LCD_move_cursor(2,8);
												LCD_send_string("on");
											}
											else
											{
												LCD_move_cursor(2,8);
												LCD_send_string("   ");
												LCD_move_cursor(2,8);
												LCD_send_string("off");
											}
											room_status=KEYBAD_user();
											if (ON_STATUS==room_status)
											{
												tv=ON;
												SPI_master_transmit_and_receive_data(TV_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(ON_STATUS);
												_delay_ms(50);
											}
											else if (OFF_STATUS==room_status)
											{
												tv=OFF;
												SPI_master_transmit_and_receive_data(TV_MENU);
												_delay_ms(50);
												SPI_master_transmit_and_receive_data(OFF_STATUS);
												_delay_ms(50);
											}
											else if (RET_STATUS==room_status)
											{
												break;
											}
										}
									}
									else if (TEMP_MENU==menu)
									{
										LCD_clr_screen();
										LCD_move_cursor(2,1);
										LCD_send_string("    Temperature");
										while(time_out<TIME_MAX)
										{
											do
											{
												SPI_master_transmit_and_receive_data(TEMP_MENU);
												_delay_ms(50);
												temperature=SPI_master_transmit_and_receive_data(DUMMY_DATA);
												_delay_ms(50);
												if (temperature<10)
												{
													LCD_move_cursor(3,7);
													LCD_send_data(' ');
													LCD_send_data(temperature+ASCII_ZERO);
													LCD_send_data(0xDF);
													LCD_send_data('C');
												}
												else if(temperature<100)
												{
													LCD_move_cursor(3,7);
													LCD_send_data(temperature/10+ASCII_ZERO);
													LCD_send_data(temperature%10+ASCII_ZERO);
													LCD_send_data(0xDF);
													LCD_send_data('C');
												}
												key_value=KEYBAD_check_press();
											} while (NOT_pressed==key_value);
											if (RET_STATUS==key_value)
											{
												break;
											}
										}	
									}
									else if (CHANGE_PASS==menu)
									{
										LCD_clr_screen();
										LCD_send_string(" set Admin password");
										LCD_move_cursor(3,1);
										LCD_send_string("password:");
										for (counter=0;counter<PASS_SIZE;counter++)
										{
											key_value=KEYBAD_user();
											LCD_send_data(key_value);
											_delay_ms(500);
											LCD_move_cursor(3,10+counter);
											LCD_send_data(PASSWORD_SYMBOL);
											EEPROM_write(EEPROM_ADMIN_ADDRESS+counter,key_value);
											
										}
										_delay_ms(500);
										LCD_clr_screen();
										LCD_move_cursor(2,1);
										LCD_send_string("   password save");
										_delay_ms(500);
										EEPROM_write(ADMIN_PASS_STATUS_ADDRESS,PASS_SET);
									}
									else if (RET_STATUS==menu)
									{
										break;
									}
								}
							}
							else if (RET_STATUS==menu)
							{
								break;
							}
						}
						LED_turn_off(ADMIN_PORT,ADMIN_PIN);
						break;
					}
				}
				break;
			}
			case GUEST :
			{
				LED_turn_on(GUEST_PORT,GUEST_PIN);
				while(time_out<TIME_MAX)
				{
					LCD_clr_screen();
					LCD_send_string("Control List:");
					LCD_move_cursor(2,1);
					LCD_send_string(" 1-Room1  2-Room2");
					LCD_move_cursor(3,1);
					LCD_send_string(" 3-Room3  4-Room4");
					LCD_move_cursor(4,1);
					LCD_send_string(" 0-back");
					menu=KEYBAD_user();
					if (ROOM1_MENU==menu)
					{
						LCD_clr_screen();
						LCD_send_string("      Room one");
						LCD_move_cursor(2,1);
						LCD_send_string("LED:");
						LCD_move_cursor(3,1);
						LCD_send_string("0-back 1-on 2-off");
						while(time_out<TIME_MAX)
						{
							if (led_R1==ON)
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("on");
							}
							else
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("off");
							}
							led_status=KEYBAD_user();
							if (ON_STATUS==led_status)
							{
								led_R1=ON;
								SPI_master_transmit_and_receive_data(ROOM1_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(ON_STATUS);
								_delay_ms(50);
							}
							else if (OFF_STATUS==led_status)
							{
								led_R1=OFF;
								SPI_master_transmit_and_receive_data(ROOM1_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(OFF_STATUS);
								_delay_ms(50);
							}
							else if (RET_STATUS==led_status)
							{
								break;
							}
						}
					}
					else if (ROOM2_MENU==menu)
					{
						LCD_clr_screen();
						LCD_send_string("      Room two");
						LCD_move_cursor(2,1);
						LCD_send_string("LED:");
						LCD_move_cursor(3,1);
						LCD_send_string("0-back 1-on 2-off");
						while(time_out<TIME_MAX)
						{
							if (led_R2==ON)
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("on");
							}
							else
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("off");
							}
							led_status=KEYBAD_user();
							if (ON_STATUS==led_status)
							{
								led_R2=ON;
								SPI_master_transmit_and_receive_data(ROOM2_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(ON_STATUS);
								_delay_ms(50);
							}
							else if (OFF_STATUS==led_status)
							{
								led_R2=OFF;
								SPI_master_transmit_and_receive_data(ROOM2_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(OFF_STATUS);
								_delay_ms(50);
							}
							else if (RET_STATUS==led_status)
							{
								break;
							}
						}
					}
					else if (ROOM3_MENU==menu)
					{
						LCD_clr_screen();
						LCD_send_string("      Room three");
						LCD_move_cursor(2,1);
						LCD_send_string("LED:");
						LCD_move_cursor(3,1);
						LCD_send_string("0-back 1-on 2-off");
						while(time_out<TIME_MAX)
						{
							if (led_R3==ON)
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("on");
							}
							else
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("off");
							}
							led_status=KEYBAD_user();
							if (ON_STATUS==led_status)
							{
								led_R3=ON;
								SPI_master_transmit_and_receive_data(ROOM3_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(ON_STATUS);
								_delay_ms(50);
							}
							else if (OFF_STATUS==led_status)
							{
								led_R3=OFF;
								SPI_master_transmit_and_receive_data(ROOM3_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(OFF_STATUS);
								_delay_ms(50);
							}
							else if (RET_STATUS==led_status)
							{
								break;
							}
						}
					}
					else if (ROOM4_MENU==menu)
					{
						LCD_clr_screen();
						LCD_send_string("      Room four");
						LCD_move_cursor(2,1);
						LCD_send_string("LED:");
						LCD_move_cursor(3,1);
						LCD_send_string("0-back 1-on 2-off");
						while(time_out<TIME_MAX)
						{
							if (led_R4==ON)
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("on");
							}
							else
							{
								LCD_move_cursor(2,5);
								LCD_send_string("   ");
								LCD_move_cursor(2,5);
								LCD_send_string("off");
							}
							led_status=KEYBAD_user();
							if (ON_STATUS==led_status)
							{
								led_R4=ON;
								SPI_master_transmit_and_receive_data(ROOM4_MENU);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(LED);
								_delay_ms(50);
								SPI_master_transmit_and_receive_data(ON_STATUS);
								_delay_ms(50);
							}
							else if (OFF_STATUS==led_status)
							{
								led_R4=OFF;
								SPI_master_transmit_and_receive_data(ROOM4_MENU);
								SPI_master_transmit_and_receive_data(LED);
								SPI_master_transmit_and_receive_data(OFF_STATUS);
							}
							else if (RET_STATUS==led_status)
							{
								break;
							}
						}
					}
					else if (RET_STATUS==menu)
					{
						break;
					}
				}
				LED_turn_off(GUEST_PORT,GUEST_PIN);
				break;
			}
			default:
			{
				break;
			}
		}
    }
}

unsigned char KEYBAD_user(void)
{
	unsigned char ret_val;
	time_out=0;
	sei(); // Set the Global Interrupt Enable bit
	do
	{
		SPI_master_transmit_and_receive_data(DUMMY_DATA);
		_delay_ms(20);
		ret_val=KEYBAD_check_press();
	} while (NOT_pressed==ret_val && time_out<TIME_MAX);
	cli(); // Clear the Global Interrupt Enable bit
	if (time_out>TIME_MAX)
	{
		LCD_clr_screen();
		LCD_move_cursor(2,1);
		LCD_send_string("      Time Out");
		_delay_ms(1000);
	}
	return ret_val;
}


ISR(TIMER0_COMP_vect)
{
	time_out++;
}