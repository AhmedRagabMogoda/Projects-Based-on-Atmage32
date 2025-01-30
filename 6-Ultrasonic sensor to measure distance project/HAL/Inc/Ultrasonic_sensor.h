/*
 * Ultrasonic_sensor.h
 *
 * Created: 8/24/2024 1:00:00 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#define F_CPU 1000000UL
#define TRIG_PIN 0
#define ECHO_PIN 6
#define SPEED_OF_SOUND 34600  // cm/s
#define PRESCALER 1

void Ultrasonic_Init(void);
unsigned int Ultrasonic_GetDistance_Polling(void);

void Ultrasonic_Init_Interrupt(void);
unsigned int Ultrasonic_GetDistance_Interrupt(void);


#endif /* ULTRASONIC_SENSOR_H_ */