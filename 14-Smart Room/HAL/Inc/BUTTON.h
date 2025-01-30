/*
 * BUTTON.h
 *
 * Created: 5/10/2024 10:38:53 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef BUTTON_H_
#define BUTTON_H_

void BUTTON_init(unsigned char port,unsigned char pin_number);

void BUTTON_init_with_pull_up(unsigned char port,unsigned char pin_number);

unsigned char BUTTON_read(unsigned char port,unsigned char pin_number);



#endif /* BUTTON_H_ */