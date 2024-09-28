# Driver2HSS86 Arduino Library

## Overview

The `Driver2HSS86` library provides an easy interface to control the HSS86 stepper motor driver using an Arduino. It allows setting up the motor, controlling its direction, speed, and position, and generating the appropriate pulses to achieve precise movement.

## Features

- Simple initialization and control of stepper motor drivers.
- Functions to set the speed (in RPM) and position (in degrees) of the motor.
- Interrupt-driven pulse generation to ensure precise timing.
- Built-in direction and enable control.

## Getting Started

### Prerequisites

- Arduino board
- HSS86 Stepper Motor Driver
- Stepper Motor

### Installation

1. Clone or download this repository.
2. Copy the `Driver2HSS86` folder into your Arduino libraries folder (`Documents/Arduino/libraries`).

### Example Usage

Here is an example of how to use the `Driver2HSS86` library in your Arduino project:

```cpp
#include <Arduino.h>
#include "Driver2HSS86.h"

// Initialize the driver (pins: pulsePin, directionPin, enablePin, pulsesPerRevolution)
Driver2HSS86 driver = Driver2HSS86::createAndAssignDriver(Pulse, Direction, Enable, PulsePerRev);

void setup() {
  // Enable the motor
  driver.enable();

  // Set speed to 100 RPM and move to 90 degrees
  driver.setSpeed(100);
  driver.setPos(100, 90);
}

void loop() {
  // Your code here
}
```
## Public Methods

### `void enable()`
Enables the stepper motor driver by setting the enable pin HIGH.

### `void disable()`
Disables the stepper motor driver by setting the enable pin LOW.

### `void setClockwise()`
Sets the motor to rotate in a clockwise direction by setting the direction pin HIGH.

### `void setCounterClockwise()`
Sets the motor to rotate in a counterclockwise direction by setting the direction pin LOW.

### `void setSpeed(float revsPerMinute)`
Sets the speed of the motor in revolutions per minute (RPM).

- **Parameters**:
  - `revsPerMinute`: Desired speed in RPM. If negative, the motor will rotate counterclockwise.

### `void setPos(float revsPerMinute, int degrees)`
Moves the motor to a specified position (in degrees) at the given RPM.

- **Parameters**:
  - `revsPerMinute`: Desired speed in RPM.
  - `degrees`: Target position in degrees. If negative, the motor will rotate counterclockwise.

## Static Utility Methods

### `static long microSecsPerPulse(float revsPerMinute, long pulsesPerRev)`
Calculates the time between pulses based on the desired RPM and pulses per revolution.

- **Parameters**:
  - `revsPerMinute`: Speed in RPM.
  - `pulsesPerRev`: Number of pulses required for one full revolution.
  
- **Returns**: Time between pulses in microseconds.

### `static int determinePrescaler(long period)`
Determines the appropriate prescaler value based on the pulse period.

- **Parameters**:
  - `period`: Pulse period in microseconds.
  
- **Returns**: The selected prescaler value.

### `static unsigned int determineCompareVal(int prescaler, long period)`
Calculates the timer compare value based on the prescaler and pulse period.

- **Parameters**:
  - `prescaler`: The selected prescaler value.
  - `period`: Pulse period in microseconds.
  
- **Returns**: The calculated compare value.
- ### `static void setPrescaler(int prescaler)`
Sets the prescaler for Timer1 based on the selected prescaler value.

- **Parameters**:
  - `prescaler`: The prescaler value to set for Timer1.


  # Timer and Interrupts

The `Driver2HSS86` library utilizes **Timer1** for generating pulses and managing movement control. Timer1 is an interrupt-driven timer, ensuring precise timing for motor control even at high speeds.

### Key Details:
- **Timer1 Compare A interrupt** (`TIMER1_COMPA_vect`): Handles the pulse generation for controlling motor speed.
- **Timer1 Compare B interrupt** (`TIMER1_COMPB_vect`): Tracks the distance to the target position and stops the motor once the desired position is reached.

Using interrupts allows the library to control the stepper motor with precise timing without relying on the `delay()` function, freeing up the CPU for other tasks.


