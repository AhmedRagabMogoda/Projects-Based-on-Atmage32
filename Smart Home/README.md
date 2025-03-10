This smart room project is designed to automate the control of devices in multiple rooms of a house. The project involves the use of two ATMEGA32 microcontrollers (Master and Slave) that communicate with each other to control different household appliances like lights, fans, air conditioning, a TV, and the home door. 

### Major Components:
1. **Master and Slave Microcontrollers (ATMEGA32)**:
   - Two ATMEGA32 microcontrollers are connected through **SPI** (Serial Peripheral Interface) for master-slave communication.
   - The master microcontroller handles user inputs and sends commands to the slave microcontroller to control various devices in the rooms.

2. **Keypad**:
   - The user interacts with the system through a keypad, which is used to enter commands. These commands could include turning devices on or off, setting the temperature, or unlocking the door.
   - The keypad may also be used to select between "Guest" and "Admin" modes.

3. **LCD Display (LM044L)**:
   - The LCD is used to display information like temperature, device status, or user credentials input.

4. **Room Control Modules** (Room One, Room Two, Room Three, Room Four):
   - Each room is equipped with devices such as a **fan** and a **light** (represented by LEDs). The fan and light are controlled individually for each room.
   - These rooms receive commands from the master-slave microcontroller interface to turn devices on or off.
   - **L293D Motor Drivers**:These are used to control the speed and direction of fans or other motors in each room.

5. **TV Control**:
   - The TV section is represented by LEDs and is controlled via the slave microcontroller. The system can turn the TV on or off based on user commands.

6. **Home Door** (Servo Motor):
   - A servo motor is used to represent the door control. The door can open (90 degrees) or close (0 degrees), likely based on user input through the keypad.

7. **Central Air Conditioning**:
   - The air conditioning system uses a **LM35 temperature sensor** to monitor the room's temperature. Depending on the temperature reading, the system will turn the air conditioner on or off.
   - The LM35 is connected to an ADC (Analog-to-Digital Converter) pin on the microcontroller for reading temperature values.

8. **EEPROM**:
   - EEPROM is used to store the states of different devices (e.g., fan, light, TV, air conditioning) so that after a reset or power failure, the system can restore the last known states of all devices.

9. **Guest/Admin LEDs**::
   - LED indicators are used to show whether the system is in guest or admin mode, restricting or allowing different levels of control for the devices.


### **System Workflow:**

1. **User Interaction**: 
   - The user interacts with the system through the keypad, either to log in as a guest/admin or to send control commands for different devices in the rooms.
   
2. **Data Processing**:
   - The master microcontroller processes the user's input and communicates with the slave microcontroller via SPI (Serial Peripheral Interface) to control the devices in different rooms.
   
3. **Device Control**:
   - Based on the commands, the system toggles the fans, lights, TV, and other devices in the rooms.
   
4. **Environmental Feedback**:
   - The air conditioning system automatically regulates itself based on the room temperature detected by the LM35 temperature sensor.
   
5. **Security/Access Control**:
   - Access control via the guest/admin mode can limit or provide access to specific features of the system.


This smart room project allows users to remotely control multiple devices in a home or room setup via an embedded system. It implements security features (guest/admin access), environmental monitoring (temperature control), and persistence (EEPROM memory). This smart room project can serve as a foundational home automation system.
