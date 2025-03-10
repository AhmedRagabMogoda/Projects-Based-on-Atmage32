/*
 * macro_function.h
 *
 * Created: 5/3/2024 12:50:32 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef MACRO_FUNCTION_H_
#define MACRO_FUNCTION_H_


#define SET_BIT(port_register,bit_number)	port_register|=(1<<bit_number)
#define	CLR_BIT(port_register,bit_number)	port_register&=(~(1<<bit_number))
#define	TOG_BIT(port_register,bit_number)	port_register^=(1<<bit_number)
#define READ_BIT(port_register,bit_number)	((port_register&(1<<bit_number))>>bit_number)

#endif /* MACRO_FUNCTION_H_ */