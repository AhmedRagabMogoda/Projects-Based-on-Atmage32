### Calculator Project Description

This project is an implementation of a basic calculator using an ATmega32 microcontroller. The calculator takes user input from a keypad and displays results on an LCD screen. It supports basic arithmetic operations such as addition, subtraction, multiplication, and division, as well as more advanced functions like square root, exponentiation, and modulus.

#### Components Used

1. **ATmega32 Microcontroller**:
   - Acts as the main processing unit for the calculator. It receives input from the keypad, processes the input to perform calculations, and outputs results to the LCD.

2. **Keypad**:
   - A matrix-style keypad (4x4) is used for user input. It allows the user to enter numbers, operations, and commands to perform calculations.

3. **LCD Display (LM016L or similar)**:
   - A 16x2 character LCD is used to display input characters, operations, and calculation results. It provides visual feedback to the user.

#### Key Functionalities

1. **Initialization**:
   - The LCD and keypad are initialized using the `LCD_init()` and `KEYBAD_init()` functions, respectively.
   - The microcontroller runs in an infinite loop to continuously wait for user input.

2. **Input Handling**:
   - **Operand Entry**: Users enter operands (numbers) and operations through the keypad.
   - **Operation Detection**: Once an operator is detected (e.g., `+`, `-`, `*`, `/`, `^`, `S` (sqrt), `%`), the input process switches to accepting the second operand if needed.
   - **Clear and Reset**: The user can reset the current calculation by pressing the `A` key.

3. **Calculation Operations**:
   - **Basic Arithmetic Operations**: Addition (`+`), subtraction (`-`), multiplication (`*`), and division (`/`).
   - **Exponentiation**: `^` operator raises the first operand to the power of the second operand.
   - **Square Root**: `S` operator computes the square root of a single operand.
   - **Modulus**: `%` operator returns the remainder of the division of the first operand by the second operand.

4. **Error Handling**:
   - **Division by Zero**: If the user attempts to divide by zero, the LCD displays an error message "Error: Division by zero."
   - **Invalid Operations**: For unsupported operations or errors in input, appropriate error messages are shown on the LCD.

5. **Result Display**:
   - The result of the calculation is displayed on the LCD screen. If the result is negative or an error occurs, it is appropriately formatted and displayed.
   - The user can chain calculations by using the result of the previous calculation as the first operand of a new operation.

#### Code Walkthrough

1. **Main Loop**:
   - The program enters an infinite loop (`while (1)`) to continuously check for user input from the keypad.
   - It waits for the first operand and operation, then proceeds to get the second operand if needed.

2. **Entering Operands**:
   - The program uses a `do-while` loop to capture numeric input from the keypad. It stores each digit entered by the user into a character array (`operand_1_str` or `operand_2_str`).

3. **Selecting an Operation**:
   - Once the first operand is entered, the program waits for an operator key press. Depending on the operator, it might require a second operand (`+`, `-`, `*`, `/`, `^`, `%`) or directly compute the result (`S` for square root).

4. **Result Calculation**:
   - Upon pressing the `=` key, the program performs the calculation based on the operator selected and operands entered.
   - A `switch` statement is used to handle the different types of operations, including error checking (e.g., division by zero or square root of a negative number).

5. **Displaying Results**:
   - The result is formatted as a string and displayed on the LCD.
   - The user can immediately use the result for further calculations by entering a new operation without starting from scratch.

6. **Continuous Operation**:
   - The program allows the user to perform multiple calculations in sequence. After displaying the result, it waits for the next operation, allowing the previous result to be used as the first operand.

#### Operational Flow

1. **Start**: Initialize LCD and keypad.
2. **Wait for First Operand**: User inputs the first number.
3. **Wait for Operation**: User selects an operation (e.g., `+`, `-`, `*`, `/`, `^`, `S`, `%`).
4. **Wait for Second Operand (if required)**: If the operation requires a second operand (e.g., `+`, `-`, etc.), the program waits for it.
5. **Perform Calculation**: The program calculates the result based on the selected operation and operands.
6. **Display Result**: Show the result on the LCD.
7. **Repeat**: Continue for further calculations or reset if the `A` key is pressed.

This project showcases a robust implementation of a calculator using embedded systems principles, allowing the user to perform various mathematical operations interactively through a keypad and view results on an LCD.
