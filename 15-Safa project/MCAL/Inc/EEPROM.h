/*
 * EEPROM.h
 *
 * Created: 7/16/2024 12:08:55 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

void EEPROM_write(unsigned short address,unsigned char data);

unsigned char EEPROM_read(const unsigned short address);



#endif /* EEPROM_H_ */