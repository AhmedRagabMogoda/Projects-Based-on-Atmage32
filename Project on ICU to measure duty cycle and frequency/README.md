This project uses the Input Capture Unit (ICU) of an AVR microcontroller to measure the duty cycle and frequency of an incoming digital signal. The ICU is a powerful feature of AVR timers that allows precise measurement of signal timing characteristics by capturing timer values at specific events (rising or falling edges of the input signal). 

### Project Description

1. **Objective**:
   - The goal of this project is to measure the duty cycle and frequency of a waveform using Timer1's Input Capture functionality on an AVR microcontroller.
   - The measured values are displayed on an LCD screen.

2. **Functionality**:
   - **Timer1** is used in Normal mode to capture timer values on the rising and falling edges of an incoming signal. The time difference between these edges allows calculation of the waveform's period and on-time (high period).
   - The duty cycle is calculated as the ratio of the on-time to the total period of the signal.
   - The frequency is calculated based on the period of the signal.

3. **Setup and Initialization**:
   - **Timer1 Configuration**:
     - Timer1 is configured in **Normal Mode** (WGM10, WGM11, WGM12, WGM13 bits are all cleared).
     - The **prescaler** is set to 1, meaning the timer clock is the same as the CPU clock (`F_CPU`).
     - **Input Capture Noise Canceller** (ICNC1) is enabled to filter out noise on the input capture pin (ICP1, PD6).
     - **Input Capture Edge Select** (ICES1) is used to select rising or falling edge detection.
   - **LCD Initialization**:
     - The LCD is initialized to display the measured duty cycle and frequency.

4. **Measurement Process**:
   - **First Rising Edge Detection**:
     - The timer value is captured on the first rising edge of the signal (`first_val`).
   - **Falling Edge Detection**:
     - The timer value is captured on the falling edge of the signal (`second_val`), which marks the end of the high period of the waveform.
   - **Second Rising Edge Detection**:
     - The timer value is captured again on the next rising edge (`third_val`), marking the end of one complete cycle of the waveform.
   - **Calculations**:
     - **On-Period** (`on_period`): Calculated as the difference between the first rising edge and the falling edge (`second_val - first_val`).
     - **Total Period** (`period`): Calculated as the difference between the first and second rising edges (`third_val - first_val`).
     - **Duty Cycle** (`duty_cycle`): Calculated as `(on_period / period) * 100`.
     - **Frequency** (`frequency`): Calculated as `F_CPU / (prescaler * period)`.

5. **Displaying Results**:
   - The calculated duty cycle and frequency are displayed on the LCD.
   - The program checks if the measurements are within a valid range to ensure accurate readings.

### Key Functions

1. **`TIMER1_ICU_init()`**:
   - Initializes Timer1 in Normal mode.
   - Sets the prescaler to 1.
   - Configures the ICP1 pin as an input.
   - Enables noise filtering on the input capture pin.
   - Clears any pending input capture flags.

2. **`TIMER1_ICU_raising()`**:
   - Configures the input capture unit to capture the timer value on a rising edge.
   - Waits for an input capture event (rising edge) and returns the captured timer value.

3. **`TIMER1_ICU_failing()`**:
   - Configures the input capture unit to capture the timer value on a falling edge.
   - Waits for an input capture event (falling edge) and returns the captured timer value.

### Calculations and Display Logic

- **Duty Cycle Calculation**:
  - The duty cycle is calculated as the proportion of the on-time to the total period.
  - It is displayed as a percentage on the LCD.

- **Frequency Calculation**:
  - The frequency is determined by the inverse of the period.
  - If the period exceeds the resolution of the timer, the program outputs "Out of Range" on the LCD.

### Key Concepts

- **Input Capture Unit (ICU)**: A feature of AVR timers that captures the value of a timer at specific events, such as rising or falling edges of an input signal. It allows precise measurement of the timing of external events.
- **Duty Cycle**: The fraction of one period in which a signal is active. It is expressed as a percentage of the total period.
- **Frequency**: The number of cycles per second of a periodic signal. It is the inverse of the period of the signal.
- **Prescaler**: A divider that reduces the clock frequency applied to the timer, allowing for longer timing intervals and finer resolution of time measurements.

By using the Input Capture Unit and Timer1, this project demonstrates how to measure and display signal characteristics accurately, which is crucial in many electronics and embedded systems applications.

### Practical Applications

This project is useful in applications where monitoring or controlling the timing characteristics of digital signals is necessary, such as:
- **PWM Signal Analysis**: Useful in motor control, LED dimming, and other applications where PWM signals are used.
- **Frequency Measurement**: Essential in communication systems, frequency counters, and signal generators.
- **Embedded System Debugging**: Helps in analyzing the timing characteristics of signals within embedded systems.
