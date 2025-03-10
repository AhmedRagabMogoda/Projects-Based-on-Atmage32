### Safe Project Description

This project involves designing an electronic safe using an ATmega32 microcontroller, a keypad for input, an LCD for feedback, and a DC motor to control the locking mechanism. The microcontroller interfaces with external EEPROM to store and verify a user-set password for accessing the safe.

#### Components Used:

1. **ATmega32 Microcontroller**: 
   - Central control unit that manages input from the keypad, interacts with the EEPROM, updates the LCD, and controls the DC motor.
   
2. **Keypad**: 
   - A numeric keypad used to input passwords for setting and verifying the code to open the safe.

3. **LCD Display (LM016L)**:
   - A 16x2 character display used to provide user feedback, such as entering and verifying passwords, or indicating if the password is correct or incorrect.

4. **DC Motor**:
   - Used to simulate the locking and unlocking mechanism of the safe. The motor turns when the correct password is entered, unlocking the safe.

5. **EEPROM (External Memory)**:
   - Stores the password securely. The EEPROM ensures that the password is retained even when power is turned off.

#### Key Functionalities:

1. **Initialization**:
   - On startup, the system initializes the LCD, keypad, and motor.
   - The system checks if a password has been set by reading a specific EEPROM address. If not set, it prompts the user to set a new password.

2. **Password Setting**:
   - The user is prompted to enter a 4-digit password using the keypad.
   - The entered password is displayed as asterisks (*) on the LCD to maintain confidentiality.
   - The password is stored in four consecutive addresses in the EEPROM.

3. **Password Verification**:
   - When the user attempts to open the safe, they must enter the 4-digit password.
   - The entered password is compared with the stored password in the EEPROM.
   - If the password matches, the safe unlocks, and the DC motor rotates to open the lock.
   - The LCDs a success message: "correct password" and "the safe opened".

4. **Changing the Password**:
   - After unlocking, the user has an option to change the password.
   - The user is prompted to enter the current password again to confirm identity, and then a new password can be set following the same procedure as the initial setup.

5. **Wrong Password Handling**:
   - The user has a maximum of three attempts to enter the correct password.
   - If a wrong password is entered, the system decrements the remaining attempts and displays "wrong password" and the number of remaining attempts.
   - After three failed attempts, the system locks out the user for some time (e.g., 60 minutes), displaying a countdown on the LCD.

6. **EEPROM Usage**:
   - **EEPROM_status_address (0x20)**: Indicates if a password has been set. Initialized to `0xFF` (default) if no password is set.
   - **EEPROM_first_address_of_password (0x21) to EEPROM_fourth_address_of_password (0x24)**: Stores each digit of the 4-digit password.

#### Code Walkthrough:

1. **Main Function**:
   - Initializes peripherals (LCD, keypad, motor).
   - Checks if a password has been set by reading `EEPROM_status_address`.
   - If no password is set, calls `set_password()` to initialize a new password.
   - Continuously loops to check if the entered password matches the stored password, allowing access and further options if correct.

2. **Set Password (`set_password` function)**:
   - Clears the LCD screen and prompts the user to set a new password.
   - Reads each digit from the keypad, displays it briefly on the LCD before replacing it with an asterisk, and stores it in EEPROM.
   - Updates the `EEPROM_status_address` to indicate that a password has been set.

3. **Check Password (`check_password` function)**:
   - Clears the LCD and prompts the user to enter the password.
   - Reads each entered digit, displays it temporarily, and compares it against the stored password.
   - Stores the entered digits in the `password_array` for further comparison.

#### Operational Flow:

1. **User Starts the System**:
   - System checks if a password is set. If not, it prompts to set a new one.
   - If a password exists, the user is prompted to enter it.

2. **User Enters the Password**:
   - The system verifies the entered password against the stored password.
   - If correct, it unlocks the safe, and the user is given options to change the password or keep it.

3. **Wrong Password Handling**:
   - If the entered password is incorrect, the user is notified and asked to try again.
   - After three incorrect attempts, the user is locked out for a specified duration.

This safe project demonstrates a secure, user-interactive embedded system using an ATmega32 microcontroller to handle digital input, provide feedback, and control an electromechanical device.
