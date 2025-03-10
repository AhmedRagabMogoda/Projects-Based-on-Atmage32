/*
 * KEYBAD.h
 *
 * Created: 7/26/2024 12:53:46 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef KEYBAD_H_
#define KEYBAD_H_
#include "DIO.h"

#define PORT_11 'D'
#define PORT_22 'D'
#define NOT_pressed 0xff

void KEYBAD_init_pull_up(void);


void KEYBAD_init(void);


char KEYBAD_check_press(void);


#endif /* KEYBAD_H_ */