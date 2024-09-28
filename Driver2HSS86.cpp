#include <Arduino.h>
#include "Driver2HSS86.h"
// Global variables for the stepper driver pins and movement tracking (ease of use/ simplicity)
extern int ena = 6;// Enable pin number for stepper motor driver
extern int pul = 5;// Pulse pin number for stepper motor driver
extern int dir = 4;// Direction pin number for stepper motor driver
extern long distFromTarget = 0;// Distance remaining to target position (in pulses)
extern long count=0;// Pulse counter for tracking progress
/**
 * @brief Constructor for the Driver2HSS86 class.
 * 
 * Initializes the stepper motor driver with the given pulses per revolution.
 * Sets the pins for pulse, direction, and enable as outputs, and ensures they start in a LOW state.
 * 
 * @param pulsesPerRevolution Number of pulses required for one full revolution of the stepper motor.
 */
Driver2HSS86::Driver2HSS86(long pulsesPerRevolution){
  pulsesPerRev = pulsesPerRevolution;  // Store pulses per revolution
  pinMode(pul, OUTPUT); // Set pulse pin as output
  pinMode(dir,OUTPUT);// Set direction pin as output
  pinMode(ena, OUTPUT); // Set enable pin as output
  digitalWrite(pul, LOW);// Set pulse pin LOW
  digitalWrite(dir, LOW);// Set direction pin LOW
  digitalWrite(ena, LOW);// Set enable pin LOW (disabled)
}
/**
 * @brief Enables the stepper motor driver by setting the enable pin HIGH.
 */
void Driver2HSS86::enable(){
  digitalWrite(ena,HIGH);// Enable the motor driver
}
/**
 * @brief Disables the stepper motor driver by setting the enable pin LOW.
 */
void Driver2HSS86::disable(){
  digitalWrite(ena,LOW); // Disable the motor driver
  
}
/**
 * @brief Set the motor to rotate in a clockwise direction.
 */
void Driver2HSS86::setClockwise(){
  digitalWrite(dir,HIGH);// Set direction pin HIGH (clockwise rotation)
}
/**
 * @brief Set the motor to rotate in a counterclockwise direction.
 */
void Driver2HSS86::setCounterClockwise(){
  digitalWrite(dir,LOW); // Set direction pin LOW (counterclockwise rotation)
}

/**
 * @brief Sets the speed of the motor in revolutions per minute (RPM).
 * 
 * Calculates the time between pulses and configures the timer with the appropriate prescaler
 * and compare value to generate pulses at the desired RPM.
 * 
 * @param revsPerMinute Desired speed in RPM.
 */
void Driver2HSS86::setSpeed(float revsPerMinute){
  // If RPM is negative, set counterclockwise rotation and convert RPM to positive
  if (revsPerMinute <= 0){
     setCounterClockwise();
     revsPerMinute *=-1;
  }else{
   
   setClockwise();// Otherwise, set clockwise rotation
  }
  // Calculate half of the pulse period (in microseconds)
  long halfOfDt = microSecsPerPulse(revsPerMinute, pulsesPerRev)/2;
  // Determine the appropriate prescaler for the calculated period
  int prescaler = determinePrescaler(halfOfDt);
  // Calculate the compare value for the timer using the prescaler
  unsigned int comp = determineCompareVal( prescaler,  halfOfDt);

  // Disable interrupts to safely modify the timer
  noInterrupts();
  setPrescaler(prescaler); // Set the timer prescaler
  TIMSK1 |= (1<<OCIE1A); // Enable Timer1 Compare A interrupt
  TCNT1 = 0; // Reset the timer counter
  OCR1A = comp;// Set the compare value
  interrupts(); // Re-enable interrupts


}
/**
 * @brief Sets the position of the motor in degrees, moving at a specified RPM.
 * 
 * The motor is moved to the desired position by calculating the required number of pulses.
 * 
 * @param revsPerMinute Speed in RPM.
 * @param degrees Desired position in degrees.
 */
void Driver2HSS86::setPos(float revsPerMinute, int degrees){
   // If degrees is negative, set counterclockwise rotation and convert degrees to positive
   if (degrees <= 0){
     setCounterClockwise();
     degrees *=-1;
  }else{
   
   setClockwise(); // Otherwise, set clockwise rotation
  }
  // Calculate half of the pulse period (in microseconds)
  long halfOfDt = microSecsPerPulse(revsPerMinute, pulsesPerRev)/2;
   // Determine the appropriate prescaler for the calculated period
  int prescaler = determinePrescaler(halfOfDt);
  // Calculate the compare value for the timer using the prescaler
  unsigned int comp = determineCompareVal( prescaler,  halfOfDt);
  // Disable interrupts to safely modify the timer
  noInterrupts();
  setPrescaler(prescaler); // Set the timer prescaler
  TIMSK1 |= (1<<OCIE1B);   // Enable Timer1 Compare B interrupt
  TCNT1 = 0; // Reset the timer counter
  OCR1B = comp; // Set the compare value
  distFromTarget = (degrees/360.00) *(pulsesPerRev);
  interrupts();


}
/**
 * @brief Calculate the time period (in microseconds) between pulses for a given RPM.
 * 
 * @param revsPerMinute Desired speed in RPM.
 * @param pulsesPerRev Number of pulses required for one revolution.
 * @return long Time between pulses in microseconds.
 */
static long Driver2HSS86::microSecsPerPulse(float revsPerMinute, long pulsesPerRev){
  long pulsesPerMinute = revsPerMinute * pulsesPerRev; // Calculate total pulses per minute
  return (60*1e6/(float)pulsesPerMinute);  // Return time between pulses in microseconds
}
/**
 * @brief Determine the appropriate prescaler based on the pulse period.
 * 
 * Chooses a prescaler to ensure the pulse period fits within the timer's limits.
 * 
 * @param period Pulse period in microseconds.
 * @return int The selected prescaler value.
 */
static int Driver2HSS86::determinePrescaler(long period){
  // Select prescaler based on the pulse period
  int prescaler = 0;
  if (period <= 32767){
    prescaler = 8;
  }else if(period <= 262140){
    prescaler = 64;
  }else if (period <= 1048560){
    prescaler =256;
  }else if (period <= 4194240){
    prescaler =1024;
  }else{
    Serial.print("The pulse signal chosen is too slow, the largest prescaler is being used");
    prescaler = 1024;
  }
  return prescaler;

}
/**
 * @brief Calculate the compare value for the timer based on the prescaler and pulse period.
 * 
 * @param prescaler The selected prescaler value.
 * @param period Pulse period in microseconds.
 * @return unsigned int The calculated compare value.
 */
static unsigned int Driver2HSS86::determineCompareVal(int prescaler, long period){
  float timePerPulse = 0;
  // Select time per clock tick based on the prescaler value
  if (prescaler == 8){
    timePerPulse = 500.00 * 1e-3;
  }else if (prescaler == 64){
    timePerPulse = 4.00;

  }else if (prescaler == 256){
    timePerPulse = 16;
    
  }
  else if (prescaler == 1024){
    timePerPulse = 64;
    
  }
  // Calculate the compare value for the timer
  uint32_t compare = period / timePerPulse;
  if (compare > 65535){
    compare =65535;
  }
  return compare;

}
/**
 * @brief Sets the prescaler for Timer1.
 * 
 * Configures Timer1 with the appropriate prescaler to control the stepper motor pulse timing.
 * 
 * @param prescaler The selected prescaler value.
 */
static void Driver2HSS86::setPrescaler(int prescaler){
   TCCR1A = 0;
   TCCR1B = 0;
   // Set the prescaler bits in the TCCR1B register based on the prescaler value
  if (prescaler == 8){
    TCCR1B |= (1<<CS11);
  }else if (prescaler == 64){
    TCCR1B |= (1<<CS11);
    TCCR1B |= (1<<CS10);

  }else if (prescaler == 256){
    TCCR1B |= (1<<CS12);
    
  }
  else if (prescaler == 1024){
     TCCR1B |= (1<<CS12);
     TCCR1B |= (1<<CS10);

    
  }
}
ISR(TIMER1_COMPA_vect){
  //reset timer and change the state of pulse pin
  TCNT1 = 0;
  PORTD ^= (1 << PORTD5);

}

ISR(TIMER1_COMPB_vect){
    TCNT1 = 0;

  //reset timer, if pin is high, set pin low
  //if off, if the distance from target is more than 0, set the pulse pin high and decrement the distance from target
  //else, it is at the target and the interupts are turned off
  if (digitalRead(5)){
    digitalWrite(5,LOW);

  }else{
    if (distFromTarget == 0){
       TIMSK1 ^= (1<<OCIE1B);
       digitalWrite(5,LOW);
    }else{
    digitalWrite(5, HIGH);
    count++;
    

    distFromTarget -= 1;
    }
  }
  
  
}

