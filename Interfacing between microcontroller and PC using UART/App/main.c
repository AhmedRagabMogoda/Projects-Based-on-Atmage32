/*
 * URAT.c
 *
 * Created: 8/26/2024 11:32:18 AM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "UART.h"
#include "LCD.h"

int main(void)
{
	UART_init(9600);
    LCD_init();
	char x;
	
    while (1) 
    {
		x=UART_receive_data();
		LCD_send_data(x);
		
    }
}

