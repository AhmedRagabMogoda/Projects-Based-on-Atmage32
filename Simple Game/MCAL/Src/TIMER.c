/*
 * TIMER.c
 *
 * Created: 8/14/2024 4:07:42 PM
 *  Author: Ahmed Ragab
 */ 

#include "TIMER.h"

void TIMER0_normal_init_with_interrupt(void)
{
	//determine normal mode
	CLR_BIT(TCCR0,WGM00);
	CLR_BIT(TCCR0,WGM01);
	//determine prescaler=1024, timer0_clock=(cpu_clock/prescaler)=8000000/1024
	SET_BIT(TCCR0,CS00);
	CLR_BIT(TCCR0,CS01);
	SET_BIT(TCCR0,CS02);
	//enable interrupt to compare match
	sei();
	SET_BIT(TIMSK,TOIE0);
	/*
	timer_tick_time=(1/timer0_clock)=0.128 ms
	occurs interrupt every 0.128*256
	*/
}

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


void TIMER2_normal_init_with_interrupt(void)
{
	//determine normal mode
	CLR_BIT(TCCR2,WGM20);
	CLR_BIT(TCCR2,WGM21);
	//determine prescaler=128, timer2_clock=(cpu_clock/prescaler)=32768/128=256HZ
	SET_BIT(ASSR,AS2);    //exit external clock=32768 HZ
	SET_BIT(TCCR2,CS20);
	CLR_BIT(TCCR2,CS21);
	SET_BIT(TCCR2,CS22);
	//enable interrupt to compare match
	sei();
	SET_BIT(TIMSK,TOIE2);
	/*
	timer_tick_time=(1/timer2_clock)=1/256 
	occurs interrupt every (1/256)*256=(1S)
	*/
}

void TIMER2_CTC_init_with_interrupt(void)
{
	//determine CTC mode
	CLR_BIT(TCCR2,WGM20);
	SET_BIT(TCCR2,WGM21);
	//load value in OCR2 ,to compare match
	OCR2=80;
	//determine prescaler=1024, timer2_clock=(cpu_clock/prescaler)
	CLR_BIT(ASSR,AS2);    //not exit external clock
	SET_BIT(TCCR2,CS20);
	SET_BIT(TCCR2,CS21);
	SET_BIT(TCCR2,CS22);
	//enable interrupt to compare match
	sei();
	SET_BIT(TIMSK,OCIE2);
	/*
	timer_tick_time=(1/timer2_clock)=0.128 ms
	occurs interrupt every 0.128*OCR0=0.128*80=(10 ms)
	*/
}
