/*
 * main_config.h
 *
 * Created: 9/5/2024 12:42:37 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

void display_clock(void);

#define DUMMY_DATA      0xff
#define ASCII_ZERO		'0'

#define ADDRESS_LED_1	0x21
#define ADDRESS_LED_2	0x22
#define ADDRESS_LED_3	0x23
#define ADDRESS_LED_4	0x24
#define ADDRESS_FAN_1	0x25
#define ADDRESS_FAN_2	0x26
#define ADDRESS_FAN_3	0x27
#define ADDRESS_FAN_4	0x28
#define ADDRESS_DOOR	0x29
#define ADDRESS_TV		0x30
#define ADDRESS_AIR_COND	0x31
#define ADDRESS_TEMP    0x32

#define ROOM1_MENU           '1'
#define ROOM2_MENU           '2'
#define ROOM3_MENU           '3'
#define ROOM4_MENU           '4'
#define HOME_DOOR_MENU		 '5'
#define TV_MENU              '6'
#define AIRCONDITIONING_MENU '7'
#define TEMP_MENU            '9' 
#define	LED '1'
#define FAN '2'  
#define ON_STATUS '1'
#define OFF_STATUS '2'
#define ON 1
#define OFF 0

#define AIR_COND_PIN				7
#define TV_PIN						6
#define led1_PIN					0
#define led2_PIN					1
#define led3_PIN					2
#define led4_PIN					3
#define fan1_PIN					4
#define fan2_PIN					5
#define fan3_PIN					6
#define fan4_PIN					7

#define AIR_COND_PORT 				'D'
#define TV_PORT       				'D'
#define DOOR_PORT       			'D'
#define ROOM1_PORT    				'C'
#define ROOM2_PORT   				'C'
#define ROOM3_PORT   				'C'
#define ROOM4_PORT    				'C'

#endif /* MAIN_CONFIG_H_ */