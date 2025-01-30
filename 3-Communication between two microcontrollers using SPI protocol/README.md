This project demonstrates how to establish communication between two AVR microcontrollers (MCUs) using the Serial Peripheral Interface (SPI) protocol. One microcontroller operates as the SPI Master, while the other functions as the SPI Slave.

**The Serial Peripheral Interface (SPI) protocol** is a synchronous serial communication protocol widely used in embedded systems for short-distance communication, primarily between microcontrollers and various peripheral devices like sensors, displays, memory chips, and more. SPI is favored for its speed, simplicity, and full-duplex communication capabilities, making it a popular choice in many embedded applications.

**Data Transfer**:
   - **Master Selects Slave**: The master selects a slave by pulling the corresponding SS line low.
   - **Clock Synchronization**: The master generates a clock signal on the SCK line. Data is synchronized with this clock signal.
   - **Data Transmission**: Data is transferred from the master to the slave via the MOSI line and from the slave to the master via the MISO line simultaneously (full-duplex communication).
   - **Data Latching**: Depending on the SPI mode (clock polarity and phase settings), data is latched on the rising or falling edge of the clock signal.
   - **Transmission Complete**: Once data transfer is complete, the master can deselect the slave by setting the SS line high.
   - **Full-Duplex Communication**:
 SPI allows simultaneous transmission and reception of data. While the master sends data to the slave via MOSI, the slave sends data to the master via MISO. This feature makes SPI a full-duplex protocol, meaning data can be transferred in both directions at the same time.


### How It Works:

 **Microcontroller One (Master):**
   - **Role:** Reads user input from a keypad and sends the data via SPI to the Slave.
   - **Functionality:** Initializes SPI as a Master and reads keypad input using the `KEYPAD_check_press()` function. Once a key is pressed, the Master transmits this data to the Slave MCU.

 **Microcontroller Two (Slave):**
   - **Role:** Receives data from the Master and displays it on an LCD.
   - **Functionality:** Initializes SPI as a Slave, receives data from the Master, and displays the received characters on an LCD screen.

####  **SPI Initialization:**

- **Master Initialization (`SPI_master_init`)**:
  - Configures the SPI pins:
    - **SS (Slave Select):** Configured as an output to control when the Slave is active.
    - **MOSI (Master Out Slave In):** Configured as an output to send data to the Slave.
    - **SCK (Serial Clock):** Configured as an output to provide the clock signal for SPI communication.
    - **MISO (Master In Slave Out):** Configured as an input to receive data from the Slave.
  - Sets the SPI mode:
    - Enables SPI, sets the MCU as the Master, and sets the clock rate to F_CPU/16.

- **Slave Initialization (`SPI_slave_init`)**:
  - Configures the SPI pins:
    - **SS, MOSI, and SCK:** Configured as inputs since the Slave receives these signals from the Master.
    - **MISO:** Configured as an output to send data back to the Master.
  - Sets the SPI mode:
    - Enables SPI and configures the MCU as the Slave.

#### **Data Transmission (Master):**

- **Keypad Input Handling**:
  - The Master reads input from the keypad using the `KEYPAD_check_press()` function.
  - A loop continuously checks for a key press. When a key is detected, it waits for a debounce delay before processing the input.
  - Once a key is detected, the Master sends the key value to the Slave using `SPI_master_transmit_and_receive_data()`.

- **Data Transmission Function (`SPI_master_transmit_and_receive_data`)**:
  - Activates the Slave by pulling the **SS** line low.
  - Sends data via the **SPDR** (SPI Data Register).
  - Waits for the transmission to complete (monitored by checking the **SPIF** flag in the **SPSR** register).
  - Deactivates the Slave by pulling the **SS** line high.

#### **Data Reception (Slave):**

- **Data Reception and Display**:
  - The Slave waits to receive data from the Master.
  - When data is received, it is automatically placed into the **SPDR**.
  - The Slave then reads this data and sends it to the LCD using `LCD_send_data()` to display the character.

- **Data Reception Function (`SPI_slave_receive_and_transmit_data`)**:
  - Loads dummy data (`0xff` or `DUMMY_DATA`) into **SPDR** to maintain the SPI clock while receiving data.
  - Waits for the transmission to complete.
  - Returns the received data from the **SPDR**.
  
### **Applications**
This project can be adapted for various applications, such as:
- **Data Logging Systems**: Where data from multiple sensors (slaves) are collected by a central microcontroller (master).
- **Multi-device Communication Networks**: In environments where multiple microcontrollers need to exchange data rapidly.
- **Data Logging and Monitoring:** By extending the project, various inputs (like sensor readings or button presses) can be logged or monitored from a central location.

Implementing this project will give you practical experience with SPI communication and microcontroller interfacing.
