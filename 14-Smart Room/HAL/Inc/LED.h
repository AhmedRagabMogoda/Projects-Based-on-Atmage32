/*
 * LED.h
 *
 * Created: 5/10/2024 4:34:52 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef LED_H_
#define LED_H_


void LED_init(unsigned char port,unsigned char pin_number);

void LED_turn_on(unsigned char port,unsigned char pin_number);

void LED_turn_off(unsigned char port,unsigned char pin_number);

void LED_toggle(unsigned char port,unsigned char pin_number);




#endif /* LED_H_ */