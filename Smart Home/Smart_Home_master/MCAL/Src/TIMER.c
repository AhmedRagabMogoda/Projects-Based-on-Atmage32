/*
 * TIMER.c
 *
 * Created: 8/14/2024 4:07:42 PM
 *  Author: Ahmed Ragab
 */ 

#include <avr/interrupt.h>
#include "macro_function.h"
#include "TIMER.h"


void TIMER0_CTC_init_with_interrupt(void)
{
	//determine CTC mode
	CLR_BIT(TCCR0,WGM00);
	SET_BIT(TCCR0,WGM01);
	//load value in OCR0 ,to compare match
	OCR0=80;
	//determine prescaler=1024, timer0_clock=(cpu_clock/prescaler)
	SET_BIT(TCCR0,CS00);
	CLR_BIT(TCCR0,CS01);
	SET_BIT(TCCR0,CS02);
	//enable interrupt to compare match
	sei();
	SET_BIT(TIMSK,OCIE0);
	/*
	timer_tick_time=(1/timer0_clock)=0.128 ms
	occurs interrupt every 0.128*OCR0=0.128*80=(10 ms)
	*/
}