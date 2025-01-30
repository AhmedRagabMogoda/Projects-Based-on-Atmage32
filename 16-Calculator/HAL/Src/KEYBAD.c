/*
 * KEYBAD.c
 *
 * Created: 7/26/2024 12:53:16 AM
 *  Author: Ahmed Ragab
 */

#include "KEYBAD.h"
#include "DIO.h"

void KEYBAD_init(void)
{
	DIO_set_pin_dir(PORT_11,0,1);
	DIO_set_pin_dir(PORT_11,1,1);
	DIO_set_pin_dir(PORT_11,2,1);
	DIO_set_pin_dir(PORT_11,3,1);
	DIO_set_pin_dir(PORT_22,0,0);
	DIO_set_pin_dir(PORT_22,1,0);
	DIO_set_pin_dir(PORT_22,2,0);
	DIO_set_pin_dir(PORT_22,3,0);
	DIO_set_pin_dir(PORT_22,4,0);
	DIO_set_pin_dir(PORT_22,5,0);
	DIO_connect_pull_up(PORT_22,0,1);
	DIO_connect_pull_up(PORT_22,1,1);
	DIO_connect_pull_up(PORT_22,2,1);
	DIO_connect_pull_up(PORT_22,3,1);
	DIO_connect_pull_up(PORT_22,4,1);
	DIO_connect_pull_up(PORT_22,5,1);
}

char KEYBAD_check_press(void)
{
	char arr_out_val[4][6]={{'A','7','8','9','*','/'},{'^','4','5','6','-','M'},{'%','1','2','3','+','B'},{'S','0','.','=','+','C'}};
	char ret_val=NOT_pressed,value;
	char row,coloum;
	for(row=0;row<4;row++)
	{
		DIO_write_pin(PORT_11,0,1);
		DIO_write_pin(PORT_11,1,1);
		DIO_write_pin(PORT_11,2,1);
		DIO_write_pin(PORT_11,3,1);
		DIO_write_pin(PORT_11,row,0);
		for (coloum=0;coloum<6;coloum++)
		{
			value=DIO_read_pin(PORT_22,coloum);
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