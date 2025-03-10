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

void TIMER0_normal_init_with_interrupt(void);

void TIMER0_CTC_init_with_interrupt(void);

void TIMER2_normal_init_with_interrupt(void);

void TIMER2_CTC_init_with_interrupt(void);

#endif /* TIMER_H_ */
