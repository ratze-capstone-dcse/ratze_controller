
# Ratze Controller

A Serial interface program designed for the Ratze micromouse robot platform. This firmware/controller code enables communication between the robot's microcontroller and a host device to support maze-solving operations

## Features
- Serial communication 
- Command interface for robot movement and maze exploration
- Logging and debugging utilities over Serial
- Ready to be integrated with maze-solving algorithms

## Requirements
- Microcontroller ESP32
- PlatformIo (Recommended to use)
- PuTTY or ArduinoIDE as the Serial Terminal program for communication

## Installation 
1. Clone this repository
```bash
git clone https://github.com/ratze-capstone-dcse/ratze_controller.git
cd ratze_controller
```
2. Open the Project in your IDE
3. Build and flash the program to your microcontroller

## Usage
1. Connect the microcontroller to PC via Serial/USB
2. Open a serial terminal with the correct baud rate (default: 115200 bps)
3. Use the available commands to move the robot

## Repository Structure
``` makefile
ratze_controller/
├── src/        # Source code for controller
├── include/    # Header files
├── lib         # Static libraries
└── README.md   # This file
```
## Authors
- RatzeDCSE Capstone Team - Hardware and Electrician

