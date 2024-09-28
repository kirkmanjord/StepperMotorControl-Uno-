# Driver2HSS86 Arduino Library

This Arduino library, `Driver2HSS86`, controls stepper motors using a HSS86 driver. It allows users to manage motor speed, direction, and position with precision, utilizing Arduino's built-in timers and interrupts for efficient pulse generation.

## Features

- Speed Control: Set the motor's speed in RPM.
- Direction Control: Rotate the motor clockwise or counterclockwise.
- Positioning: Move the motor to a specified angular position.
- Pulse Timer Management: Uses Arduino's Timer1 to generate accurate pulse signals for motor control.
- Efficient Interrupt Handling: Leverages interrupts to track and control motor movement without blocking the main loop.

## Requirements

- Arduino Board: Compatible with Arduino Uno.
- Stepper Motor Driver: Designed specifically for HSS86 driver.
- Stepper Motor: Works with stepper motors that use pulse signals for control.

## Pin Configuration

The library assumes the following pin configuration for controlling the stepper motor driver:

- ENA (Enable): Pin 6
- PUL (Pulse): Pin 5
- DIR (Direction): Pin 4

## Installation

1. Download or clone the `Driver2HSS86` library.
2. Copy the folder to your Arduino libraries directory: `{Arduino Sketchbook}/libraries/`.
3. Include the library in your project by adding `#include "Driver2HSS86.h"` at the beginning of your sketch.

## Example Usage

#include <Arduino.h>
#include "Driver2HSS86.h"

// Create a driver object with 200 pulses per revolution
Driver2HSS86 stepper(200);

void setup() {
  // Initialize the motor
  stepper.enable();
  
  // Set the speed to 60 RPM and move the motor to 90 degrees
  stepper.setSpeed(60);
  stepper.setPos(60, 90);
}

void loop() {
  // Continuous movement logic can go here
}

## API Reference

### Constructor

Driver2HSS86(long pulsesPerRevolution);
- pulsesPerRevolution: Number of pulses required for one full revolution of the motor.

### Methods

- void enable(): Enables the stepper motor driver.
- void disable(): Disables the stepper motor driver.
- void setClockwise(): Sets the motor direction to clockwise.
- void setCounterClockwise(): Sets the motor direction to counterclockwise.
- void setSpeed(float revsPerMinute): Sets the motor's speed in revolutions per minute (RPM).
- void setPos(float revsPerMinute, int degrees): Moves the motor to a specified position in degrees at a given speed (RPM).

## How It Works

- Speed Control: The `setSpeed()` function calculates the time between each pulse based on the desired RPM and uses Timer1 to generate those pulses.
- Positioning: The `setPos()` function converts the target position in degrees into the equivalent number of pulses, then tracks the remaining distance to the target as pulses are generated.

## Interrupt Handling

The library uses two interrupts:

- TIMER1_COMPA_vect: Toggles the pulse signal for continuous movement.
- TIMER1_COMPB_vect: Decrements the pulse count for precise positioning and stops the motor when the target is reached.

## Notes

- Ensure that the pin configuration matches your hardware setup.
- Adjust the number of pulses per revolution according to your motor's specifications.

## License

This library is open-source and provided under the MIT license. Feel free to modify and distribute it as needed.
