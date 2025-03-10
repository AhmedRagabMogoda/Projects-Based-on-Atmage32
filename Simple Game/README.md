### Description of the Simple Math Game Project

This project is a simple math game implemented on an AVR microcontroller. The game consists of a series of math problems, with the player required to enter the correct answer within a given time limit (10 seconds). The game uses an LCD to show the math problems and results, a keypad to receive input from the user, and Timer0 to manage the timing.

#### Project Components

1. **LCD Display**: Used to display math problems, user input, and game messages.
2. **Keypad**: Allows the user to input their answers to the math problems.
3. **Timer0**: Configured in CTC (Clear Timer on Compare Match) mode with an interrupt to manage the timing of each level.
4. **Game Logic**: Handles user input, checks answers, and controls the flow of the game.

#### Key Features

1. **Multiple Levels**: The game presents different math problems in each level, and the player needs to solve them correctly to progress to the next level.
2. **Time Limit**: Each level has a 10-second time limit for the player to input the correct answer.
3. **Scoring System**: The player earns points for each correct answer, and the score is displayed after each level.
4. **Interrupt-Driven Timing**: Timer0's CTC mode with interrupts is used to measure the 10-second time limit for each level.
5. **Game Over Conditions**: The game ends if the player fails to answer within the time limit or provides an incorrect answer.

#### Game Logic:
  - The game presents three math puzzles sequentially to the player.
  - Each puzzle is displayed on the LCD screen with a missing number represented by "??".
  - The player has to input the correct number using the keypad before the time limit (10 seconds) for each level expires.
  - The player score is updated based on the correctness and timeliness of their answers.

#### Level Progression:
  - If the player solves a puzzle correctly within the time limit, they proceed to the next level.
  - If the player answers incorrectly or runs out of time, the game ends, and the final score is displayed.
    
#####  **Timer and Interrupt Handling**

- **Timer0 CTC Mode**: Configured to generate an interrupt every 10 ms using the formula `OCR0 = 80` with a prescaler of 1024. This provides a timer tick of 0.128 ms and a compare match every `80 * 0.128 ms = 10.24 ms`, approximately 10 ms.
- **Interrupt Service Routine (ISR)**: `ISR(TIMER0_COMP_vect)` increments a counter (`flag1`) every 10 ms. When `flag1` reaches 1000 (equivalent to 10 seconds), the game detects a timeout, setting `flag2` to indicate time over.

#### Flow of the Game

1. **Start the Game**: Display the first problem and initialize the timer.
2. **Player Input**: The player enters their answer using the keypad.
3. **Check Time Limit**: The program checks if 10 seconds have passed.
4. **Evaluate Answer**: 
   - If correct and within time, proceed to the next level.
   - If incorrect or out of time, display the appropriate message and end the game.
5. **Next Levels**: Repeat the process for the remaining problems.


This project is a simple yet effective demonstration of using an AVR microcontroller to create an interactive game with a keypad and LCD interface, incorporating real-time timing constraints using Timer0 and interrupts. It provides a foundational understanding of how to integrate peripherals and interrupts in embedded systems to create a user-interactive application.


