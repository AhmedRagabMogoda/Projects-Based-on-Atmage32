/*
 * DC_MOTOR.c
 *
 * Created: 5/12/2024 7:51:09 PM
 *  Author: Ahmed Ragab
 */ 


#include <avr/io.h>
#include "DIO.h"
#include "DC_MOTOR.h"

void DC_motor_init(void)
{
	DIO_set_pin_dir(MotorPort,MotorPin1,1);
	DIO_set_pin_dir(MotorPort,MotorPin2,1);
}

void DC_motor_Forward(void)
{
	DIO_write_pin(MotorPort,MotorPin1,1);
	DIO_write_pin(MotorPort,MotorPin2,0);

}

void DC_motor_Backward(void)
{
	DIO_write_pin(MotorPort,MotorPin1,0);
	DIO_write_pin(MotorPort,MotorPin2,1);	
	  
}

void DC_motor_Stop(void) 
{
	DIO_write_pin(MotorPort,MotorPin1,0);
	DIO_write_pin(MotorPort,MotorPin2,0);
}
