/*
 * EEPROM.c
 *
 * Created: 7/16/2024 12:08:29 AM
 *  Author: Ahmed Ragab
 */ 


#include <avr/io.h>
#include "macro_function.h"


void EEPROM_write(unsigned short address,unsigned char data)
{
	
	EEARL=(char)address;
	EEARH=(char)(address>>8);
	EEDR=data;
	SET_BIT(EECR,EEMWE);
	SET_BIT(EECR,EEWE);
	while(1==READ_BIT(EECR,EEWE));
}

unsigned char EEPROM_read(const unsigned short address)
{
	
	EEARL=(char)address;
	EEARH=(char)(address>>8);
	SET_BIT(EECR,EERE);
	return EEDR;
}