/*
 * Generate_Waveforms.c
 *
 * Created: 8/17/2024 1:43:36 AM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#include "TIMER.h"

int main(void)
{
   TIMER0_CTC_wave_nonpwm();
   TIMER1_fast_pwm_8bit_wave();
   TIMER2_phase_correct_pwm_wave();
}

