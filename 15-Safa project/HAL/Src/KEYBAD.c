/*
 * KEYBAD.c
 *
 * Created: 7/26/2024 12:53:16 AM
 *  Author: Ahmed Ragab
 */

#include "KEYBAD.h"
#include "DIO.h"

void KEYPAD_init(void)
{
	DIO_set_pin_dir(PORT,0,1);
	DIO_set_pin_dir(PORT,1,1);
	DIO_set_pin_dir(PORT,2,1);
	DIO_set_pin_dir(PORT,3,1);
	DIO_set_pin_dir(PORT,4,0);
	DIO_set_pin_dir(PORT,5,0);
	DIO_set_pin_dir(PORT,6,0);
	DIO_set_pin_dir(PORT,7,0);
	DIO_connect_pull_up(PORT,4,1);
	DIO_connect_pull_up(PORT,5,1);
	DIO_connect_pull_up(PORT,6,1);
	DIO_connect_pull_up(PORT,7,1);
}

char KEYPAD_read(void)
{
	char arr_out_val[4][4]={{'7','8','9','/'},{'4','5','6','*'},{'1','2','3','-'},{'A','0','=','+'}};
	char ret_val=0xff,value;
	char row,coloum;
	for(row=0;row<4;row++)
	{
		DIO_write_pin(PORT,0,1);
		DIO_write_pin(PORT,1,1);
		DIO_write_pin(PORT,2,1);
		DIO_write_pin(PORT,3,1);
		DIO_write_pin(PORT,row,0);
		for (coloum=0;coloum<4;coloum++)
		{
			value=DIO_read_pin(PORT,(coloum+4));
			if(0==value)
			{
				ret_val=arr_out_val[row][coloum];
				break;
			}
		}
		if (0==value)
		{
			break;
		}
	}
	return ret_val;
}