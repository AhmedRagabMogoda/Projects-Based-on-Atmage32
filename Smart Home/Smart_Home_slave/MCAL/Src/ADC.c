/*
 * ADC.c
 *
 * Created: 8/8/2024 2:21:15 AM
 *  Author: Ahmed Ragab
 */ 
#include <avr/io.h>
#include "macro_function.h"

void ADC_init(void)
{
	//enable ADC
	SET_BIT(ADCSRA,ADEN);
	//determine pin connected ADC PA0
	CLR_BIT(ADMUX,MUX0);
	CLR_BIT(ADMUX,MUX1);
	CLR_BIT(ADMUX,MUX2);
	CLR_BIT(ADMUX,MUX3);
	CLR_BIT(ADMUX,MUX4);
	//configurion voltage internal 2.56v
	SET_BIT(ADMUX,REFS0);
	SET_BIT(ADMUX,REFS1);
	//configurion clock 8MHZ/64=125KHZ
	CLR_BIT(ADCSRA,ADPS0);
	SET_BIT(ADCSRA,ADPS1);
	SET_BIT(ADCSRA,ADPS2);
}

unsigned short ADC_u16read(void)
{
	unsigned short ret_read;
	SET_BIT(ADCSRA,ADSC); //start conversion
	while (1==READ_BIT(ADCSRA,ADSC));
	/*OR*/
	/*
	while (0==READ_BIT(ADCSRA,ADIF));
	SET_BIT(ADCSRA,ADIF);
	*/
	ret_read=ADCL;
	ret_read|=(ADCH<<8);
	return ret_read;
}
