/*
 * main_config.h
 *
 * Created: 9/5/2024 12:42:37 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

#define DUMMY_DATA 0xff
#define TIME_MAX 600

#define TRUE 0
#define FALSE 1
#define PASS_SIZE      4
#define TRIES_ALLOWED   3
#define ADDRESS_TRIES  0x20
#define ADDRESS_TRIES_count  0x21
#define TIME_CLOSED    59
#define PASSWORD_SYMBOL '*'
#define ASCII_ZERO      '0'

#define PASS_NOT_SET 0xFF
#define PASS_SET     0x01

#define ADMIN_PASS_STATUS_ADDRESS 0X101
#define EEPROM_ADMIN_ADDRESS      0X102

#define ADMIN '1'
#define GUEST '2'
#define ADMIN_PORT 'B'
#define GUEST_PORT 'B'
#define ADMIN_PIN   0
#define GUEST_PIN   1

#define MORE_MENU            'A'
#define ROOM1_MENU           '1'
#define ROOM2_MENU           '2'
#define ROOM3_MENU           '3'
#define ROOM4_MENU           '4'
#define HOME_DOOR			 '5'
#define TV_MENU              '6'
#define AIRCONDITIONING_MENU '7'
#define CHANGE_PASS			 '8'
#define TEMP_MENU			 '9'
#define	LED '1'
#define FAN '2' 
#define ON 1
#define OFF 0  
#define ON_STATUS '1'
#define OFF_STATUS '2'
#define RET_STATUS '0'

#endif /* MAIN_CONFIG_H_ */