This project demonstrates how to generate four different waveforms using three timers (Timer0, Timer1, and Timer2) on an AVR microcontroller. Each timer is configured for a specific mode to generate waveforms with different characteristics, such as Clear Timer on Compare Match (CTC) and Pulse Width Modulation (PWM). The project also shows how to calculate the frequency of these generated waveforms.

### Project Description

1. **Timer Configurations**:
   - The project uses three timers (Timer0, Timer1, and Timer2) in different modes to generate waveforms on output pins.
   - **Timer0**: Configured in CTC (Clear Timer on Compare Match) mode to generate a non-PWM waveform.
   - **Timer1**: Configured in Fast PWM 8-bit mode to generate two PWM waveforms on separate output pins.
   - **Timer2**: Configured in Phase Correct PWM mode to generate a PWM waveform.

2. **Waveform Generation**:
   - **CTC Mode (Timer0)**:
     - **Configuration**: Timer0 is set to CTC mode with a prescaler of 1024.
     - **Waveform Output**: A square wave is generated on the `OC0` pin (PB3). The output toggles whenever the timer value matches the value in `OCR0` (Output Compare Register 0).
     - **Frequency Calculation**:
       - The timer's tick time is calculated as `timer_tick_time = 1 / timer0_clock`, where `timer0_clock = cpu_clock / prescaler`.
       - With a prescaler of 1024, if `cpu_clock` is 8 MHz, `timer0_clock = 8,000,000 / 1024 = 7812.5 Hz`.
       - `timer_tick_time = 1 / 7812.5 = 0.000128 s` (or 0.128 ms).
       - The pulse period is `0.128 * OCR0`. With `OCR0 = 80`, the pulse period is `0.128 * 80 = 10.24 ms`.
       - The frequency is `1 / period = 1 / 10.24 ms ≈ 97.66 Hz`.

   - **Fast PWM Mode (Timer1)**:
     - **Configuration**: Timer1 is set to Fast PWM mode with an 8-bit resolution and a prescaler of 1024.
     - **Waveform Output**: Two PWM waveforms are generated on pins `OC1A` (PD5) and `OC1B` (PD4). The duty cycle of each PWM signal is set by `OCR1A` and `OCR1B`.
     - **Frequency Calculation**:
       - `timer_tick_time = 1 / timer1_clock`.
       - With a prescaler of 1024, `timer1_clock = cpu_clock / 1024 = 7812.5 Hz`.
       - `timer_tick_time = 1 / 7812.5 ≈ 0.000128 s` (or 0.128 ms).
       - The PWM frequency for an 8-bit timer is determined by the top value (256 for an 8-bit timer), so the PWM period is `0.128 * 256 = 32.768 ms`.
       - The frequency is `1 / 32.768 ms ≈ 30.52 Hz`.

   - **Phase Correct PWM Mode (Timer2)**:
     - **Configuration**: Timer2 is set to Phase Correct PWM mode with a prescaler of 1024.
     - **Waveform Output**: A PWM waveform is generated on pin `OC2` (PD7). The duty cycle of the PWM signal is set by `OCR2`.
     - **Frequency Calculation**:
       - `timer_tick_time = 1 / timer2_clock`.
       - With a prescaler of 1024, `timer2_clock = cpu_clock / 1024 = 7812.5 Hz`.
       - `timer_tick_time = 1 / 7812.5 ≈ 0.000128 s` (or 0.128 ms).
       - In Phase Correct PWM mode, the counter counts up to 255 and then down to 0, so the period is `2 * 255 * timer_tick_time`.
       - Period = `2 * 255 * 0.128 ms = 65.28 ms`.
       - The frequency is `1 / 65.28 ms ≈ 15.32 Hz`.

3. **Functions Explanation**:
   - **`TIMER0_CTC_wave_nonpwm()`**: Configures Timer0 in CTC mode.
     - Clears `WGM00` and sets `WGM01` to enter CTC mode.
     - Sets the compare value (`OCR0 = 80`) to control the frequency of the waveform.
     - Configures prescaler to 1024 and sets the output pin `OC0` (PB3) to toggle on compare match.

   - **`TIMER1_fast_pwm_8bit_wave()`**: Configures Timer1 in Fast PWM mode.
     - Sets `WGM10` to enable 8-bit Fast PWM mode.
     - Sets compare values (`OCR1A = 64` and `OCR1B = 64`) to control the duty cycle of PWM signals.
     - Configures prescaler to 1024 and sets output pins `OC1A` (PD5) and `OC1B` (PD4) with different compare match behaviors.

   - **`TIMER2_phase_correct_pwm_wave()`**: Configures Timer2 in Phase Correct PWM mode.
     - Sets `WGM20` to enter Phase Correct PWM mode.
     - Sets the compare value (`OCR2 = 64`) to control the duty cycle of the PWM signal.
     - Configures prescaler to 1024 and sets the output pin `OC2` (PD7) to clear on compare match when counting up and set on compare match when counting down.

### Key Concepts

- **CTC Mode (Clear Timer on Compare Match)**: Generates a waveform where the output pin toggles or clears based on the timer reaching a predefined value, useful for creating non-PWM square waves.
  
- **Fast PWM Mode**: Generates a high-frequency PWM signal by quickly cycling the output between high and low states. The duty cycle is controlled by comparing the timer count to a predefined value.

- **Phase Correct PWM Mode**: Generates a PWM signal where the output changes only at the end of each count (up or down), resulting in a symmetrical waveform and reducing harmonic distortion.

- **Prescaler**: Divides the system clock frequency to adjust the timer's operating frequency. Higher prescaler values result in slower timer ticks.

### Practical Applications

This project demonstrates the use of AVR microcontroller timers to generate various waveforms, which are critical in applications like motor control, LED dimming, signal generation, and communication protocols. Understanding these concepts is essential for designing embedded systems that require precise timing and waveform generation.
