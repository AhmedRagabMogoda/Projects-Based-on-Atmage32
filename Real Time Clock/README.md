### Description of the Real-Time Clock (RTC) Project

This project is a Real-Time Clock (RTC) implemented on an AVR microcontroller. It features a user interface for setting the clock using a keypad and displays the current time on a seven-segment display. The project utilizes Timer2 for keeping track of seconds, and it supports hours, minutes, and seconds display.

#### Project Components

1. **LCD Display**: Used for user interactions, such as setting the time.
2. **Keypad**: Allows the user to input the hour, minute, and second values to set the clock.
3. **Seven-Segment Display**: Shows the current time in hours, minutes, and seconds.
4. **Timer2**: Manages timekeeping by generating interrupts every second.

#### Key Features

1. **Time Setting**: Users can set the hour, minute, and second values using the keypad.
2. **Real-Time Clock Display**: The current time is continuously displayed on the seven-segment display.
3. **Timer Interrupt**: Timer2 is used to increment the seconds counter every second, which in turn updates the minutes and hours.

#### Detailed Explanation

##### 1. **Initialization**

- **LCD Initialization**: `LCD_init()` initializes the LCD display for interaction.
- **Keypad Initialization**: `KEYBAD_init()` sets up the keypad for user input.
- **Seven-Segment Display Initialization**: `seven_seg_init_port()` prepares the port connected to the seven-segment display.
- **Timer2 Initialization**: `TIMER2_normal_init_with_interrupt()` configures Timer2 in normal mode with a prescaler to generate a 1-second interrupt.

##### 2. **Setting Clock**:
  - Pressing 'A' triggers setting the clock time.
  - Sequentially input hours, minutes, and seconds via the keypad.
    
##### 3. **Displaying Clock**:
- **Multiplexing Technique**: Multiplexing involves sequentially turning on one seven-segment display at a time while quickly switching between them to give the illusion that all displays are on simultaneously.
- **Multiplexing Implementation**: The code cycles through each digit of the time (hours, minutes, seconds) and sends the corresponding data to the seven-segment display before moving to the next digit.
- Segments are updated in a rapid sequence to create the illusion of continuous display.
 
##### 4. **Timekeeping**

- **Timer2 Interrupt**: Timer2 generates an interrupt every second (1s) to update the seconds counter:
  - `ISR(TIMER2_OVF_vect)` increments the `seconds_counter` every second.

- **Display Update**: The `display_clock()` function:
  - Updates the seven-segment display to show the current hours, minutes, and seconds.
  - Rotates through the seven-segment display positions to show each time unit.
  - Handles rollover of seconds to minutes, and minutes to hours:
    - If `seconds_counter` reaches 60, it resets to 0 and increments `minutes_counter`.
    - If `minutes_counter` reaches 60, it resets to 0 and increments `hours_counter`.
    - If `hours_counter` reaches 24, it resets to 0.

##### 5. **Timer2 Configuration**

- **Timer Mode**: Configured in normal mode with no waveform generation (WGM20 and WGM21 are cleared).
- **Prescaler**: Set to 128 to achieve a timer clock frequency of 256 Hz (based on a 32.768 kHz external clock source). This configuration results in a timer overflow interrupt every second.
- **Interrupt Enable**: Timer overflow interrupt is enabled with `SET_BIT(TIMSK, TOIE2)`.

This project demonstrates a basic real-time clock application using an AVR microcontroller. It incorporates:
- **User Input**: For setting the time through a keypad.
- **Timekeeping**: Managed by Timer2 and displayed on a seven-segment display.
- **Interrupts**: To keep track of the time with a 1-second resolution.

This RTC project provides a practical example of integrating various components and peripherals to create a functional timekeeping system with user interaction.
