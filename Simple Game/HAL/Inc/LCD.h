/*
 * LCD.h
 *
 * Created: 7/24/2024 2:01:49 AM
 *  Author: Ahmed Ragab
 */ 


#ifndef LCD_H_
#define LCD_H_
#include "DIO.h"

#define four_bits_mode
#define PORT_1 'A'
#define PORT_2 'B'
#define CLR_SCREEN 0x01
#define CURSOR_ON_DISPLAY_ON 0x0E
#define CURSOR_Off_DISPLAY_ON 0x0c
#define RETURN_HOME 0x02
#define ENTRY_MODE 0x06

#if defined eight_bits_mode
#define RS 0
#define RW 1
#define EN 2
#define EIGHT_BITS 0X38
#elif defined four_bits_mode
#define RS 0
#define RW 1
#define EN 2
#define FOUR_BITS 0X28
#endif

void LCD_init(void);
void LCD_send_cmd(char cmd);
void LCD_send_data(char data);
void LCD_send_string(char* string);
void LCD_clr_screen(void);
void LCD_move_cursor(char row,char coloum);



#endif /* LCD_H_ */