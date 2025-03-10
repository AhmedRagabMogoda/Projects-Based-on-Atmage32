The temperature meter project uses an AVR microcontroller to measure and display the temperature on an LCD screen. It utilizes an Analog-to-Digital Converter (ADC) to read the analog voltage from a temperature sensor and converts this reading into a temperature value, which is then displayed on an LCD.
### Project Description

1. **Microcontroller and ADC Setup**:
   - The project uses an AVR microcontroller with an ADC module to read the analog voltage from a temperature sensor. The ADC converts the analog voltage signal to a digital value that the microcontroller can process.
   - The reference voltage for the ADC is set to 2.56V, and the precision is defined as 1024 levels (10-bit resolution), meaning the ADC can output a value from 0 to 1023.

2. **Constants and Calculations**:
   - **Reference Voltage**: The `reference_voltage` macro is defined as 2.56V, which is the reference voltage supplied to the ADC.
   - **Precision**: The `precision` macro represents the number of discrete levels the ADC can distinguish, which is `1024` for a 10-bit ADC.
   - **Resolution**: The `resolution` is calculated as `reference_voltage/precision`, which determines the smallest voltage increment the ADC can detect.
   - The resolution is then multiplied by `1000` to convert the voltage from volts to millivolts (mV), making it easier to work with typical temperature sensor outputs.

3. **LCD and ADC Initialization**:
   - The LCD is initialized using `LCD_init()`, setting up the display for output.
   - The ADC is initialized with `ADC_init()`, preparing the microcontroller to read analog inputs.

4. **Temperature Measurement and Display**:
   - The program enters an infinite loop to continuously read the temperature and display it on the LCD.
   - **Voltage Reading**: The ADC value is read using `ADC_u16read()`, which provides a digital value proportional to the analog input voltage from the temperature sensor. This value is then converted to millivolts (mV) by multiplying by the `resolution` and `1000`.
   
   - **Temperature Calculation**:
     - The temperature is calculated based on the voltage output of the sensor. Assuming a sensor where `10 mV` corresponds to `1°C`, the temperature is calculated differently depending on whether the voltage is above or below `1000 mV`:
     - The temperature is calculated based on the voltage:
     - **Positive Temperatures**: 
       - If the voltage (`volt`) is greater than or equal to 1000 mV, the temperature in degrees Celsius is calculated using the formula `(volt - 1000) / 10`. This assumes that every 10 mV corresponds to 1 degree Celsius above 0°C.
     - **Negative Temperatures**:
       - If the voltage is less than 1000 mV, it implies a negative temperature. The temperature is then calculated using `(1000 - volt) / 10`. This indicates that for every 10 mV below 1000 mV, the temperature decreases by 1 degree Celsius.
       
   - **LCD Display**:
    **Display Temperature on LCD**:
      - The temperature is displayed on the LCD in the format "Temperature=X°C", where X is the calculated temperature value.
      - If the temperature is negative, a minus sign (`'-'`) is displayed before the temperature value on the LCD.

5. **Continuous Reading**:
   - The while loop ensures continuous temperature monitoring and updating of the display.

### Key Concepts

- **ADC (Analog-to-Digital Converter)**: Converts the analog voltage from the temperature sensor into a digital value that can be processed by the microcontroller.
  
- **Temperature Sensor Output**: Many temperature sensors output a voltage that corresponds linearly to temperature (e.g., 10 mV per degree Celsius), and this project uses that relationship to compute the temperature.

- **Real-Time LCD Display Update**: The project continuously updates the LCD display to show the current temperature, providing real-time feedback.

### Practical Applications

This project demonstrates how to measure environmental parameters such as temperature using an analog sensor and a microcontroller. The ability to display this information on an LCD makes it practical for creating standalone devices like digital thermometers, HVAC controllers, or any embedded system requiring temperature monitoring.
