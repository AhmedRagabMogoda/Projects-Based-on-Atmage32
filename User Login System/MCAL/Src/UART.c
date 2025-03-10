/*
 * UART.c
 *
 * Created: 8/26/2024 11:54:48 AM
 *  Author: Ahmed Ragab
 */ 

#define F_CPU 8000000UL
#include <util/delay.h>  
#include <avr/io.h>
#include "macro_function.h"

void UART_init(unsigned long baud)
{
	//choose baud rate 
	unsigned short UBRR;
	UBRR=(F_CPU/(baud*16))-1;
	UBRRH=(unsigned char)(UBRR>>8);
	UBRRL=(unsigned char)UBRR;
	//enable USART receiver and sender
	SET_BIT(UCSRB,TXEN);
	SET_BIT(UCSRB,RXEN);
	// choose number of data bits and stop bits and parity
	UCSRC=(1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
	/*
	asynchronous normal mode
	number of data bits=8
	stop bits=1
	no parity
	*/
}

void UART_send_data(char data)
{
	//wait for UDR transmit buffer to be empty
	while(READ_BIT(UCSRA,UDRE)==0);
	//write data to UDR transmi buffer transmit
	UDR=data;
}

void UART_send_string(char *ptr)
{
	while((*ptr) != '\0')
	{
		UART_send_data(*ptr);
		ptr++;
		_delay_ms(100);
	}
}

char UART_receive_data(void)
{
	//wait for UDR receive buffer to be filled with data
	while(READ_BIT(UCSRA,RXC)==0);
	//receive data from UDR receive buffer
	return UDR;
}
