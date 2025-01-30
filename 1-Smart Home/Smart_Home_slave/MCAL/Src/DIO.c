/*
 * DIO.c
 *
 * Created: 5/3/2024 12:49:24 AM
 *  Author: Ahmed Ragab
 */ 

#include <avr/io.h>
#include "macro_function.h"

void DIO_set_pin_dir(unsigned char port,unsigned char bit_number,unsigned char direction)
{
	switch(port)
	{
		case 'A':
		case 'a':
		if(direction==1)
		{
			SET_BIT(DDRA,bit_number);
		}
		else
		{
			CLR_BIT(DDRA,bit_number);
		}
		break;
		case 'B':
		case 'b':
		if(direction==1)
		{
			SET_BIT(DDRB,bit_number);
		}
		else
		{
			CLR_BIT(DDRB,bit_number);
		}
		break;
		case 'C':
		case 'c':
		if(direction==1)
		{
			SET_BIT(DDRC,bit_number);
		}
		else
		{
			CLR_BIT(DDRC,bit_number);
		}
		break;
		case 'D':
		case 'd':
		if(direction==1)
		{
			SET_BIT(DDRD,bit_number);
		}
		else
		{
			CLR_BIT(DDRD,bit_number);
		}	
		break;
		default:					
		break;
	} 
}
void DIO_write_pin(unsigned char port,unsigned char bit_number,unsigned char output_value)
{
	switch(port)
	{
		case 'a':
		case 'A':
		if(output_value==1)
		{
			SET_BIT(PORTA,bit_number);
		}
		else
		{
			CLR_BIT(PORTA,bit_number);
		}
		break;
		case 'b':
		case 'B':
		if(output_value==1)
		{
			SET_BIT(PORTB,bit_number);
		}
		else
		{
			CLR_BIT(PORTB,bit_number);
		}
		break;
		case 'c':
		case 'C':
		if(output_value==1)
		{
			SET_BIT(PORTC,bit_number);
		}
		else
		{
			CLR_BIT(PORTC,bit_number);
		}
		break;		
		case 'd':
		case 'D':
		if(output_value==1)
		{
			SET_BIT(PORTD,bit_number);
		}
		else
		{
			CLR_BIT(PORTD,bit_number);
		}
		break;	
		default:
		break;			
		
	}
}

void DIO_toggle_pin(unsigned char port ,unsigned char bit_number)
{
	switch(port)
	{
		case 'a':
		case 'A':
		TOG_BIT(PORTA,bit_number);
		break;
		case 'b':
		case 'B':
		TOG_BIT(PORTB,bit_number);
		break;
		case 'c':
		case 'C':
		TOG_BIT(PORTC,bit_number);
		break;		
		case 'd':
		case 'D':
		TOG_BIT(PORTD,bit_number);
		break;		
		default:
		break;		
	}
}

unsigned char DIO_read_pin(unsigned char port,unsigned char bit_number)
{
	unsigned char ret_value=0;
	switch(port)
	{
		case 'a':
		case 'A':
		ret_value=READ_BIT(PINA,bit_number);
		break;
		case 'B':
		case 'b':
		ret_value=READ_BIT(PINB,bit_number);
		break;
		case 'c':
		case 'C':
		ret_value=READ_BIT(PINC,bit_number);
		break;				
		case 'd':
		case 'D':
		ret_value=READ_BIT(PIND,bit_number);
		break;	
		default:
		break;	
	}
	return ret_value;
}

void DIO_set_port_dir(unsigned char port,unsigned char direction)
{
	switch(port)
	{
		case 'a':
		case 'A':
		DDRA=direction;
		break;
		case 'b':
		case 'B':
		DDRB=direction;
		break;		
		case 'c':
		case 'C':
		DDRC=direction;
		break;		
		case 'd':
		case 'D':
		DDRD=direction;
		break;
		default:
		break;		
	}
	
}

void DIO_write_port(unsigned char port,unsigned char output_value)
{
	switch(port)
	{
		case 'a':
		case 'A':
		PORTA=output_value;
		break;
		case 'b':
		case 'B':
		PORTB=output_value;
		break;		
		case 'c':
		case 'C':
		PORTC=output_value;
		break;		
		case 'd':
		case 'D':
		PORTD=output_value;
		break;	
		default:
		break;	
		
	}
}

void DIO_toggle_port(unsigned char port)
{
	switch(port)
	{
			case 'a':
			case 'A':
			PORTA=~PORTA;
			break;
			case 'b':
			case 'B':
			PORTB=~PORTB;
			break;		
			case 'c':
			case 'C':
			PORTC=~PORTC;
			break;		
			case 'd':
			case 'D':
			PORTD=~PORTD;
			break;
			default:
			break;		
	}	
}

unsigned char DIO_read_port(unsigned char port)
{
	unsigned char ret_value=0;
	switch(port)
	{
		case 'a':
		case 'A':
		ret_value=PINA;
		break;
		case 'b':
		case 'B':
		ret_value=PINB;
		break;		
		case 'c':
		case 'C':
		ret_value=PINC;
		break;		
		case 'd':
		case 'D':
		ret_value=PIND;
		break;
		default:
		break;		
		
	}
	return ret_value;
}

void DIO_connect_pull_up(unsigned char port,unsigned char pin_number,unsigned char connect_pull_up)
{
	switch(port)
	{
		case 'a':
		case 'A':
		if(1==connect_pull_up)
		{
			SET_BIT(PORTA,pin_number);
		}
		else
		{
			CLR_BIT(PORTA,pin_number);
		}
		break;
		case 'b':
		case 'B':
		if(1==connect_pull_up)
		{
			SET_BIT(PORTB,pin_number);
		}
		else
		{
			CLR_BIT(PORTB,pin_number);
		}
		break;		
		case 'c':
		case 'C':
		if(1==connect_pull_up)
		{
			SET_BIT(PORTC,pin_number);
		}
		else
		{
			CLR_BIT(PORTC,pin_number);
		}
		break;		
		case 'd':
		case 'D':
		if(1==connect_pull_up)
		{
			SET_BIT(PORTD,pin_number);
		}
		else
		{
			CLR_BIT(PORTD,pin_number);
		}
		break;		
		default:
		break;
	}
}

void DIO_write_low_nibble(unsigned char port, unsigned char value )
{
	value&=0x0F;
	switch(port)
	{
		case 'A':
		case 'a':
		PORTA&=0xf0;
		PORTA|=value;
		break;
		case 'B':
		case 'b':
		PORTB&=0xf0;
		PORTB|=value;
		break;
		case 'C':
		case 'c':
		PORTC&=0xf0;
		PORTC|=value;
		break;
		case 'D':
		case 'd':
		PORTD&=0xf0;
		PORTD|=value;
		break;
		default:
		break;
	}
}

void DIO_write_high_nibble(unsigned char port,unsigned char number)
{
	number<<=4;
	switch(port)
	{
		case 'A':
		case 'a':
		PORTA&=0x0F;
		PORTA|=number;
		break;
		case 'B':
		case 'b':
		PORTB&=0x0F;
		PORTB|=number;
		break;
		case 'C':
		case 'c':
		PORTC&=0x0F;
		PORTC|=number;
		break;
		case 'D':
		case 'd':
		PORTD&=0x0F;
		PORTD|=number;
		break;
		default:
		break;
	}
	
	
}