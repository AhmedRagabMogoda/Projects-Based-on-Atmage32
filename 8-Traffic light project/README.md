 Traffic Light using AVR Atmega32 with 7-Segment Display,7-Segment Decoder, LCD, and Traffic Lights.
 The system uses LEDs to represent the green, yellow, and red lights and a seven-segment display showing each light's countdown. Additionally, an LCD is used to display messages such as "GO," "Ready," and "STOP."

Key Components:
1. **LEDs**:
   - Three LEDs are connected to pins `C0`, `C1`, and `C2`, representing green, yellow, and red lights, respectively.

2. **Seven-Segment Display**:
   - A seven-segment display is used to show the countdown for each light. The display is divided into a low nibble and a high nibble, allowing the display of two-digit numbers.

3. **LCD Display**:
   - An LCD is used to show messages to indicate the current state of the traffic light ("GO," "Ready," and "STOP").

4. **Timer**:
   - The project uses Timer0 in CTC (Clear Timer on Compare Match) mode with an interrupt to keep track of time. The timer increments a counter variable that is used to control the timing of the lights and the seven-segment display.

5. **Counter**:
   - A `volatile` counter variable is incremented in the Timer0 interrupt service routine (ISR). When the counter reaches 100 (which corresponds to a certain time interval), the respective light’s countdown is decreased, and the seven-segment display is updated.

### Functionality:
1. **Initialization**:
   - The LCD, seven-segment display, timer, and LEDs are initialized. The microcontroller pins are configured for output where necessary.
2. **Traffic Light Sequence**:
  - Green Phase:
Turns on the green LED, displays "GO....." on the LCD, and counts down from 30 to 0 while updating the value on the 7-segment displays.
  - Yellow Phase:
Turns off green LED, turns on yellow LED, displays "Ready" on the LCD, and counts down from 5 to 0 while updating the 7-segment displays.
  - Red Phase:
Turns off yellow LED, turns on red LED, displays "STOP" on the LCD, and counts down from 20 to 0 while updating the 7-segment displays.
3. **Timer0 ISR**:
   - The Timer0 ISR increments the `counter` variable. When the counter reaches 100, it signals that one second has passed, and the countdown in the main loop is decremented.

### Potential Use Cases:
- **Traffic Light Simulation**: This project could be used in a lab environment to simulate a traffic light system and understand the timing and control logic behind traffic signals.
