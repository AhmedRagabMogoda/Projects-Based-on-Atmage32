/*
 * TIMER.h
 *
 * Created: 8/14/2024 4:08:00 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef TIMER_H_
#define TIMER_H_
#include "macro_function.h"
#include <avr/interrupt.h>

void TIMER1_ICU_init(void);

unsigned short TIMER1_ICU_raising(void);

unsigned short TIMER1_ICU_failing(void);

void TIMER1_ICU_init_with_interrupt(void);

#endif /* TIMER_H_ */
