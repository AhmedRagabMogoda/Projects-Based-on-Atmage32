This project uses an AVR microcontroller to measure distance using an ultrasonic sensor. The setup involves generating ultrasonic pulses, capturing the echo response, and calculating the distance based on the time it takes for the echo to return. The distance measurement is displayed on an LCD screen. Below is a detailed explanation of how the project works:

### **Project Components and Setup**

1. **Microcontroller**: AVR microcontroller (e.g., ATmega32) is used for controlling and processing.
2. **Ultrasonic Sensor**: The sensor emits ultrasonic sound waves and listens for their echo. The time taken for the echo to return is proportional to the distance of the object.
3. **LCD Display**: Used to show the measured distance or a message indicating no object is detected.
4. **Timer and Interrupts**: Timer1 in the microcontroller is configured to capture the time at the rising and falling edges of the echo signal. This timer configuration helps in accurately measuring the time of flight of the ultrasonic pulse.

### **Key Macros and Constants**

- **`F_CPU 1000000UL`**: Defines the CPU frequency as 1 MHz.
- **`prescaling 1`**: Timer1 prescaler set to 1, meaning the timer clock is equal to the CPU clock.
- **`speed_of_sound 34600`**: Speed of sound in air is approximately 34600 cm/s, used for distance calculation.

### **Program Workflow**

1. **Initialization**:
   - **LCD_init()**: Initializes the LCD display.
   - **TIMER1_ICU_init()**: Configures Timer1 for input capture to measure the echo pulse width accurately.
   
2. **Triggering the Ultrasonic Sensor**:
   - A short 10 µs pulse is sent to the ultrasonic sensor to trigger it and start the distance measurement process.

3. **Capture Rising and Falling Edge**:
   - The `TIMER1_ICU_raising()` function captures the time value (`ICR1`) when the echo signal first arrives (rising edge).
   - The `TIMER1_ICU_failing()` function captures the time value when the echo signal ends (falling edge).

4. **Calculate Pulse Width and Distance**:
   - **Pulse Width Calculation**: The pulse width, which is the duration between the rising and falling edges, is calculated as `on_pulse = failing_value - raising_value`.
   - **Time Calculation**: The time for which the pulse was high is calculated using `time_on_pulse = on_pulse * (prescaling / F_CPU)`.
   - **Distance Calculation**: The distance to the object is calculated using the formula `distance = (speed_of_sound * time_on_pulse) / 2`. The division by 2 is because the sound wave travels to the object and back (round trip).

5. **Display the Distance**:
   - If the calculated distance is greater than 80 cm, it indicates "No object" on the LCD.
   - If an object is detected within 80 cm, it displays the distance in centimeters on the LCD.

6. **Timer Functions**:
   - **`TIMER1_ICU_init()`**: Initializes Timer1 in normal mode and sets up input capture on the ICP1 pin with noise filtering.
   - **`TIMER1_stop_ICU()`**: Stops Timer1 by clearing the control registers.
   - **`TIMER1_ICU_raising()` and `TIMER1_ICU_failing()`**: Functions to measure the rising and falling edges of the echo signal, respectively.

### **Detailed Code Walkthrough**
  
- **Timer1 Configuration**:
  - Timer1 is configured in normal mode without any special PWM or CTC modes (`TCCR1A` and `TCCR1B` control registers).
  - The prescaler is set to 1, meaning Timer1 increments at the same rate as the CPU clock.
  - Input capture is enabled on the ICP1 pin with noise canceling enabled (`ICNC1` bit in `TCCR1B`).

- **Input Capture**:
  - The `ICES1` bit in `TCCR1B` determines whether the input capture occurs on a rising or falling edge.
  - `ICF1` flag in `TIFR` is used to check if a capture event has occurred.

### **Usage and Application**

This ultrasonic sensor project is ideal for applications where distance measurement is required, such as:

- **Obstacle Detection**: Used in robotics for detecting obstacles.
- **Distance Measurement**: Used in projects where you need to measure the distance of objects from a point, such as parking sensors in cars.
- **Level Measurement**: Can be used to measure the level of liquids in a tank.

By accurately calculating the distance using the time-of-flight of sound waves, this project demonstrates a simple yet effective way to utilize ultrasonic sensors with microcontrollers for various real-world applications.
