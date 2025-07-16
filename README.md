# Remote-Control-Car
# Overview
A wireless remote control car project using STM32 microcontrollers and NRF24L01+ modules for communication. The system consists of a transmitter (controller) and a receiver (car) that can operate in both manual and autonomous modes.

## Features

### Transmitter (Controller)
- Joystick control for direction (up, down, left, right)
- Button inputs for additional functions
- LCD display showing speed and movement status
- Wireless communication via NRF24L01+
- RGB LED indicators

### Receiver (Car)
- Manual control mode (receives commands from transmitter)
- Autonomous mode with obstacle detection
- Four proximity sensors (front, back, left, right)
- L298N motor driver control
- Wireless communication via NRF24L01+
- Status LED indicators

## Hardware Components
- STM32 microcontrollers (both transmitter and receiver)
- NRF24L01+ wireless modules
- Joystick module (transmitter)
- Push buttons (transmitter)
- I2C LCD display (transmitter)
- L298N motor driver (receiver)
- DC motors with wheels (receiver)
- IR proximity sensors (receiver)
- RGB LEDs (transmitter)
- Status LEDs (both)

## Project Structure
- `Transmitter.c`: Controller code (reads inputs, sends commands)
- `Receive.c`: Car code (receives commands, controls motors)

## Notes
- The project uses 2.5GHz frequency for wireless communication
- Default data rate is 1Mbps
- Addresses are set to "node0" and "node1" for the two devices
- The system includes basic collision avoidance in autonomous mode

## Future Improvements
- Add battery level monitoring
- Implement more sophisticated autonomous navigation
- Add speed control via PWM
- Include status feedback to transmitter
- Implement OTA firmware updates
