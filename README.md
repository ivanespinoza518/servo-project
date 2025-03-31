# Embedded Servo Control and Display System

## Overview
This project is an embedded application for controlling a servo motor based on a potentiometer input while displaying the current values on an I2C-based LCD screen. It utilizes FreeRTOS on an ESP32 microcontroller to manage multitasking and real-time data updates.

## Features
- Reads analog input from a potentiometer.
- Converts potentiometer values into PWM signals to control a servo motor.
- Displays potentiometer and servo values on an I2C LCD.
- Implements FreeRTOS tasks for concurrent processing.

## Hardware Requirements
- **ESP32 Development Board**
- **Servo Motor** (PWM controlled)
- **10kΩ Potentiometer**
- **I2C LCD Display (PCF8574 based, address 0x3F)**
- **Wiring Connections:**
  - Potentiometer:
    - VCC → GPIO 27
    - GND → GPIO 32
    - Signal → GPIO 25
  - Servo:
    - Signal → GPIO 15
  - I2C LCD:
    - SDA → GPIO 21
    - SCL → GPIO 22

## Software Components
### 1. `main.c`
- Initializes FreeRTOS queues.
- Configures ADC for reading potentiometer values.
- Sets up PWM for servo control.
- Implements two FreeRTOS tasks:
  - **Servo Task**: Receives potentiometer values and adjusts the servo position.
  - **Display Task**: Receives values and updates the LCD screen.

### 2. `i2c-lcd.c` & `i2c-lcd.h`
- Implements functions to initialize and control the I2C LCD display.
- Provides functions to send commands, write strings, and position the cursor.

## Expected Behavior
- Turning the potentiometer changes the servo motor's position.
- The LCD displays the real-time potentiometer value and corresponding servo PWM signal.

