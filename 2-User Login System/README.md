User login system project with atmega32 and external EEPROM. This project enhances a user authentication system built on an AVR microcontroller platform, integrating an external EEPROM for persistent storage, an LCD for display feedback, and a UART for user input. The system allows initial username and password setup and provides a basic settings menu where users can change their username or password after logging in successfully. The use of external EEPROM allows the system to retain user data even after power is lost, ensuring reliability and persistence.


### I2C Communication with EEPROM:

I2C protocol is employed to communicate with an external EEPROM chip where the user data (username and password) is stored. This setup allows data persistence across power cycles, ensuring that user credentials are not lost when the system is turned off.

*I²C (Inter-Integrated Circuit)* is a serial communication protocol commonly used in embedded systems to enable communication between a master device (like a microcontroller) and one or more slave devices (such as sensors, EEPROMs, or other microcontrollers) over a two-wire interface. I²C is widely used due to its simplicity, flexibility, and ability to support multiple devices on the same bus

**How I²C Works:**

*Start Condition*:
   - Communication begins when the master issues a *Start* condition. This is done by pulling the SDA line low while the SCL line is high.

 *Address Frame*:
   - The master sends a 7-bit or 10-bit address on the SDA line, followed by a single bit indicating the direction of data transfer (write or read).
   - The addressed slave device acknowledges receipt of its address by pulling the SDA line low during the acknowledgment bit.

 *Data Transfer*:
   - If the master is writing data to the slave, it continues to send bytes on the SDA line. Each byte must be acknowledged by the slave.
   - If the master is reading data from the slave, the slave sends data bytes on the SDA line, and the master acknowledges receipt after each byte, except the last one.

 *Stop Condition*:
   - The master ends communication by issuing a *Stop* condition, which is done by releasing the SDA line to high while SCL is high.


### *UART Communication between the microcontroller and pc:*

UART is utilized for serial communication between the microcontroller and a terminal or computer, allowing users to input their username and password. This method simplifies user interaction and is suitable for development and testing phases.

*UART (Universal Asynchronous Receiver/Transmitter)* is a widely used serial communication protocol in embedded systems for asynchronous data exchange between two devices, such as microcontrollers, computers, sensors, or other peripherals. Unlike synchronous communication protocols like I²C or SPI, UART does not require a shared clock signal between the transmitting and receiving devices, making it simpler and more versatile for point-to-point communication.

*Data Transmission*:
   - When the transmitter is ready to send data, it pulls the TX (transmit) line low to signal the start bit.
   - The transmitter sends the data bits one at a time, starting with the least significant bit (LSB).
   - If a parity bit is enabled, it is sent after the data bits.
   - Finally, the transmitter sends one or two stop bits to signal the end of the frame.
   - Between transmissions, the TX line remains high (idle state).

 *Data Reception*:
   - The receiver continuously monitors the RX (receive) line for a start bit (a transition from high to low).
   - Upon detecting a start bit, the receiver begins sampling the RX line at regular intervals corresponding to the configured baud rate.
   - The receiver reconstructs the data bits from the sampled values.
   - It then checks the optional parity bit for errors.
   - Finally, the receiver verifies the stop bits to ensure the frame's integrity.
     
### Key Components and Functionalities:

1. *Microcontroller (AVR)*:
   - The AVR microcontroller is the central control unit, managing the interface with the EEPROM, LCD, and UART modules. It coordinates reading and writing operations to the EEPROM, displays messages and prompts on the LCD, and receives user input through UART.

2. *LCD Display*:
   - The LCD is used to display prompts and messages to the user, such as instructions for setting up a username and password, login status, and the settings menu options. The LCD serves as the primary user interface.

3. *UART Communication*:
   - UART is utilized for serial communication between the microcontroller and a terminal or computer, allowing users to input their username and password. This method simplifies user interaction and is suitable for development and testing phases.

4. *I2C Communication with EEPROM*:
   - I2C protocol is employed to communicate with an external EEPROM chip where the user data (username and password) is stored. This setup allows data persistence across power cycles, ensuring that user credentials are not lost when the system is turned off.

5. *EEPROM Usage*:
   - EEPROM (Electrically Erasable Programmable Read-Only Memory) stores the username and password, making them non-volatile. Specific addresses in EEPROM are dedicated to storing each character of the username and password, and a status address is used to determine if the EEPROM is in its default state.

6. *Initial Setup*:
   - When the system is powered on, it checks if the EEPROM is in its default state by reading a status byte. If it is, the system prompts the user to create a username and password. These are stored in EEPROM, and the EEPROM status is updated to indicate that the system has been initialized.

7. *Login Process*:
   - After the initial setup, the system prompts users to log in by entering their username and password. The entered data is compared with the stored data in EEPROM. If the credentials match, a successful login message is displayed; otherwise, appropriate error messages are shown.

8. *Settings Menu*:
   - Upon successful login, the user is presented with a settings menu that allows changing the username or password:
   - *Change Password*: Prompts the user to enter a new password, which is then updated in the EEPROM.
   - *Change Username*: Prompts the user to enter a new username, which is then updated in the EEPROM.

### Detailed Code Functionality:

1. *Initialization*:
   - The microcontroller initializes the LCD, UART, and I2C communication modules. A brief delay is introduced to ensure all components are ready for operation.

2. *Checking EEPROM State*:
   - The system reads a specific EEPROM address (EEPROM_status_address) to determine if the EEPROM is set to its default value (0xFF). If the EEPROM is in the default state, it initiates the user creation process.

3. *Creating Username and Password*:
   - The system prompts the user to create a new username and password via UART input, displays the input on the LCD, and writes the data to specific EEPROM addresses using I2C communication.

4. *Marking EEPROM as Initialized*:
   - Once the username and password are stored, the system updates the EEPROM status address to indicate that initialization is complete, preventing the setup prompt from appearing on future startups.

5. *Login Procedure*:
   - The user is prompted to log in by entering their stored username and password. The system reads the stored credentials from EEPROM and compares them with the input. If they match, the user gains access; otherwise, the system displays an error message and halts further operations.

6. *Settings and Configuration*:
   - After a successful login, the system presents the user with a settings menu:
   - *Change Password*: Allows the user to update the password stored in EEPROM.
   - *Change Username*: Allows the user to update the username stored in EEPROM.
   - Each selection involves clearing the LCD, prompting the user for new input, and writing the new data to EEPROM.

### Potential Applications:

- *Secure Access Control*: This project can be used for simple access control systems requiring user authentication, such as in labs, restricted rooms, or prototype environments.
- *User Configuration Management*: Useful in embedded systems where configuration data (usernames and passwords) must persist across power cycles.
- *Basic User Management*: A prototype for more complex systems where user credentials need to be managed and modified securely.


This project provides a foundational understanding of building a basic secure access system using embedded systems principles, offering room for growth in complexity and functionality.
