/*
 * TIMER.c
 *
 * Created: 8/14/2024 4:07:42 PM
 *  Author: Ahmed Ragab
 */ 

#include "TIMER.h"


void TIMER1_ICU_init(void)
{
	//determine normal mode
	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	CLR_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13);
	//determine prescaler=1, timer1_clock=(cpu_clock/prescaler)
	SET_BIT(TCCR1B,CS10);
	CLR_BIT(TCCR1B,CS11);
	CLR_BIT(TCCR1B,CS12);
	//set ICP1 as input pin to received wave
	CLR_BIT(DDRD,PD6);
	//filter noise on input capure pin
	SET_BIT(TCCR1B,ICNC1);
	//clear flag
	SET_BIT(TIFR,ICF1); 
}

unsigned short TIMER1_ICU_raising(void)
{
	unsigned short val;
	//take capure when raising
	SET_BIT(TCCR1B,ICES1);
	// Wait for capture event and Read ICR1
	while(READ_BIT(TIFR,ICF1)==0);
	val=ICR1;
	SET_BIT(TIFR,ICF1); //clear flag
	return val;
	/*
	timer_tick_time=(1/timer1_clock)
	*/
}

unsigned short TIMER1_ICU_failing(void)
{
	unsigned short val;
	//take capure when failing
	CLR_BIT(TCCR1B,ICES1);
	// Wait for capture event and Read ICR1
	while(READ_BIT(TIFR,ICF1)==0);
	val=ICR1;
	SET_BIT(TIFR,ICF1); //clear flag
	return val;
	/*
	timer_tick_time=(1/timer1_clock)
	*/
}

void TIMER1_ICU_init_with_interrupt(void)
{
	//determine normal mode
	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	CLR_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13);
	//determine prescaler=1, timer1_clock=(cpu_clock/prescaler)
	SET_BIT(TCCR1B,CS10);
	CLR_BIT(TCCR1B,CS11);
	CLR_BIT(TCCR1B,CS12);
	//set ICP1 as input pin to received wave
	CLR_BIT(DDRD,PD6);
	//filter noise on input capure pin
	SET_BIT(TCCR1B,ICNC1);
	//take capure when raising
	SET_BIT(TCCR1B,ICES1);
	//enable interrupt when capture event
	sei();
	SET_BIT(TIMSK,TICIE1);
}
