/*
 * ADC.h
 *
 * Created: 8/8/2024 2:21:34 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef ADC_H_
#define ADC_H_

#define reference_voltage 2.56     //voltage of ADC
#define precision 1024            //number of levels=2^n   n is bits
#define resolution (reference_voltage/precision)

void ADC_init(void);

unsigned short ADC_u16read(void);


#endif /* ADC_H_ */