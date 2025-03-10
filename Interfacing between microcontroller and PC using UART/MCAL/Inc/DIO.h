/*
 * DIO.h
 *
 * Created: 5/3/2024 12:48:58 AM
 *  Author: Ahmed Ragab
 */ 
/*
#include <avr/interrupt.h>
#define F_CPU 8000000UL
#include <util/delay.h>  
*/

#ifndef DIO_H_
#define DIO_H_

void DIO_set_pin_dir(unsigned char port,unsigned char bit_number,unsigned char direction);

void DIO_write_pin(unsigned char port,unsigned char bit_number,unsigned char output_value);

void DIO_toggle_pin(unsigned char port ,unsigned char bit_number);

unsigned char DIO_read_pin(unsigned char port,unsigned char bit_number);

void DIO_set_port_dir(unsigned char port,unsigned char direction);

void DIO_write_port(unsigned char port,unsigned char output_value);

void DIO_toggle_port(unsigned char port);

unsigned char DIO_read_port(unsigned char port);

void DIO_connect_pull_up(unsigned char port,unsigned char pin_number,unsigned char connect_pull_up);

void DIO_write_low_nibble(unsigned char port,unsigned char number );

void DIO_write_high_nibble(unsigned char port,unsigned char number);

#endif /* DIO_H_ */