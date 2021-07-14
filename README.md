# MicroMario
C implementation of the classic game for stm32f303 - Microcontrollers and Assembly Language project, spring 2021

## Overview
This project uses the following external modules on an STM32F3 Discovery board:
 1. 20×4 LCD
 2. Relay (for jump sound effects)
 3. Volume Controller (for game speed control)
 4. Seven-Segment display and a correponding 7447 IC (for score and speed display)
 5. UART connector for message relaiance and gameplay control
 6. 4×4 Keypad (for gameplay control)
 
 Note that the game utilises the buttons and integrated LED lights of the Dicovery board.
 
 ## Gameplay
 The program includes a main splash screen, after which a single-level game with 60 cells of randomly generated map is loaded (random seed is given by the exact time the player starts the game after splash screen). 
 Mario can be controlled using a serially connected terminal and the buttons 'D' and Space to run/jump. Similarly, it can be controlled using the keypad module as per the keymap provided in `src/main.c`.
 'P' can be used for pausing the game and 'R' for start/resume. The game includes various graphics for pause/game loss/game win.
 
 Obstacles include pipes and blocks that limit Mario's movement, while the occassional pit or hitting the map's edge blades can result in loss of life. Mario enjoys a 3 iteration (depending on game speed) immunity to blades and pits after he loses a life.
 Mario can also collect overhead chests by hitting them from downwards, in which case they generate up to 3 coins. In the event of Mario's death (loss of all 3 lives), the score = coins, but if Mario reaches the flag at 60th cell, the score is multiplied by an inverse coefficient of the gameplay duration.
 
 ## Notes
 * In theory, the volume controller can change the map's scroll speed between 0.5 - 2 cells per second, however, due to LCD rendering issues, the actual speed cannot reaach 2 c/s.
 * To interface the LCD module, the [STM32LiquidCrystal](https://github.com/SayidHosseini/STM32LiquidCrystal) library has been used.
 * Code generation for processor configurations has been done in STM32CubeMXIDE, and the code heavily utilises HAL library functions. It is recommended to use STM32CubeMXIDE or a combination of CLion and STM32CubeMX for code generation.
