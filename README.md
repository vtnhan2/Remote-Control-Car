# Remote-Control-Car

## Overview
This project implements a wireless robot control system using NRF24L01+ modules for communication between a transmitter (controller) and receiver (robot). The system features joystick control, obstacle detection, and multiple operation modes.

## System Components

### Transmitter (Controller)
- **Inputs**:
  - Analog joystick (X/Y axes)
  - 8 digital buttons
- **Features**:
  - Converts joystick movements to directional commands
  - Sends control data wirelessly via NRF24L01+
  - LCD display for status feedback
  - Button configuration for special commands

### Receiver (Robot)
- **Outputs**:
  - 4 DC motors (L298N driver)
  - Obstacle detection sensors (4-directional)
  - Status LEDs
- **Features**:
  - Receives and processes wireless commands
  - Motor control with speed/direction adjustment
  - Obstacle detection and collision prevention
  - Automatic and manual operation modes

## Key Features

- **Wireless Communication**:
  - NRF24L01+ 2.4GHz RF modules
  - 2500kbps data rate
  - Custom address configuration ("node0"/"node1")

- **Control Modes**:
  - **Manual Mode**: Direct joystick control
  - **Automatic Mode**: Autonomous movement with obstacle avoidance

- **Movement Commands**:
  - Forward/Backward (W/S)
  - Left/Right turns (A/D)
  - Diagonal movements (L/R)
  - 8-speed levels (0-7)

- **Safety Features**:
  - Immediate stop on obstacle detection
  - Sensor feedback to controller
  - Emergency stop capability

## Hardware Setup

### Transmitter Components:
- STM32 microcontroller
- NRF24L01+ module
- Analog joystick
- 8 push buttons
- I2C LCD display
- RGB status LED

### Receiver Components:
- STM32 microcontroller
- NRF24L01+ module
- L298N motor driver
- 4 DC motors (with mecanum wheels)
- 4 IR obstacle sensors (front, back, left, right)
- Status LEDs
- Buzzer for alerts

## Communication Protocol

### Data Packet Structure (4 bytes):
1. **Control Byte**: Movement direction (W/S/A/D/R/L/n)
2. **Speed Byte**: Speed level (0-7)
3. **Button Byte**: Button presses (0-8)
4. **Enable Byte**: Activation flag (0/1)

### Sensor Feedback:
- Single byte indicating obstacle direction (N/R/L/B/S)

## Usage Instructions

1. **Power on both transmitter and receiver**
2. **Controller LCD will show**:
   - Current speed (0-7)
   - Movement direction
   - Button presses
   - Obstacle alerts
3. **Control the robot**:
   - Joystick: Movement direction
   - Buttons: Special commands
   - Button 8: Toggle auto/manual mode
4. **Obstacle handling**:
   - Robot stops automatically when detecting obstacles
   - Controller LED changes color when obstacles are detected

## Code Structure

### Transmitter (Transmitter.c)
- **Main Components**:
  - Joystick reading and processing
  - Button scanning
  - LCD display output
  - NRF24L01+ transmission
  - Data packet assembly

### Receiver (Receive.c)
- **Main Components**:
  - NRF24L01+ reception
  - Motor control (PWM outputs)
  - Sensor monitoring
  - Mode switching
  - Buzzer alerts


The project demonstrates wireless control, sensor integration, and real-time feedback in an embedded system.
