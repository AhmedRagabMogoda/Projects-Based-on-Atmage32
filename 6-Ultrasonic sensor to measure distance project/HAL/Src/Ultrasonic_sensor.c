/*
 * Ultrasonic_sensor.c
 *
 * Created: 8/24/2024 1:00:00 PM
 *  Author: Ahmed Ragab
 */ 

#include "Ultrasonic_sensor.h"
#include "macro_function.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


void Ultrasonic_Init(void)
{
	// Configure Trigger pin as output
	SET_BIT(DDRD,TRIG_PIN);
	// Configure Echo pin as input
	CLR_BIT(DDRD,ECHO_PIN);

	//determine normal mode
	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	CLR_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13);
	//determine prescaler=1, timer1_clock=(cpu_clock/prescaler)
	SET_BIT(TCCR1B,CS10);
	CLR_BIT(TCCR1B,CS11);
	CLR_BIT(TCCR1B,CS12);
	//filter noise on input capure pin
	SET_BIT(TCCR1B,ICNC1);
}

unsigned int Ultrasonic_GetDistance_Polling(void)
{
	unsigned int rising_time, falling_time, pulse_width;
	float time_on_pulse;
	unsigned int distance;

	// Generate 10µs trigger pulse
	SET_BIT(PORTD,TRIG_PIN);
	_delay_us(10);
	CLR_BIT(PORTD,TRIG_PIN);

	// Wait for rising edge
	TCCR1B |= (1 << ICES1);  // Capture rising edge
	TIFR |= (1 << ICF1);     // Clear flag
	while (!(TIFR & (1 << ICF1))); // Wait for event
	rising_time = ICR1;      // Store timestamp
	TIFR |= (1 << ICF1);     // Clear flag

	// Wait for falling edge
	TCCR1B &= ~(1 << ICES1); // Capture falling edge
	while (!(TIFR & (1 << ICF1))); // Wait for event
	falling_time = ICR1;     // Store timestamp
	TIFR |= (1 << ICF1);     // Clear flag

	// Compute pulse width
	pulse_width = falling_time - rising_time;
	time_on_pulse = pulse_width * ((float) PRESCALER / F_CPU);

	// Compute distance cm
	distance = (SPEED_OF_SOUND * time_on_pulse) / 2;

	return distance;
}

/********** Ultrasonic Function With Interrupts ************************/

volatile uint16_t rising_time = 0;
volatile uint16_t falling_time = 0;
volatile uint8_t flag = 0;

void Ultrasonic_Init_Interrupt(void)
{
	// Configure Trigger pin as output
	SET_BIT(DDRD,TRIG_PIN);
	// Configure Echo pin as input
	CLR_BIT(DDRD,ECHO_PIN);
	//determine normal mode
	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	CLR_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13);
	//determine prescaler=1, timer1_clock=(cpu_clock/prescaler)
	SET_BIT(TCCR1B,CS10);
	CLR_BIT(TCCR1B,CS11);
	CLR_BIT(TCCR1B,CS12);
	//filter noise on input capure pin
	SET_BIT(TCCR1B,ICNC1);
	// Set input capture edge select to rising edge
	TCCR1B |= (1 << ICES1);
	// Enable Input Capture Interrupt
	TIMSK |= (1 << TICIE1);
	// Enable global interrupts
	sei();
}

unsigned int Ultrasonic_GetDistance_Interrupt(void)
{
	unsigned int pulse_width;
	float time_on_pulse;
	unsigned int distance;

	// Generate 10µs trigger pulse
	SET_BIT(PORTD,TRIG_PIN);
	_delay_us(10);
	CLR_BIT(PORTD,TRIG_PIN);

	// Wait for measurement to complete
	while (flag < 2);

	// Compute pulse width
	pulse_width = falling_time - rising_time;
	time_on_pulse = pulse_width * ((float) PRESCALER / F_CPU);

	// Compute distance
	distance = (SPEED_OF_SOUND * time_on_pulse) / 2;

	// Reset flag for next measurement
	flag = 0;

	return distance;
}

ISR(TIMER1_CAPT_vect)
{
	if (flag == 0)  // Rising edge detected
	{
		rising_time = ICR1;
		flag = 1;
		TCCR1B &= ~(1 << ICES1); // Set for falling edge
	}
	else if (flag == 1)  // Falling edge detected
	{
		falling_time = ICR1;
		flag = 2; // Measurement complete
		TCCR1B |= (1 << ICES1);  // Reset for next measurement
	}
}