#ifndef DRIVER2HSS86_H
#define DRIVER2HSS86_H
#include <Arduino.h>

// External global variables for pulse, direction, enable, and distance tracking
extern int pul;// Pulse pin number for stepper motor driver
extern int dir;// Direction pin number for stepper motor driver
extern int ena;// Enable pin number for stepper motor driver
extern long distFromTarget; // Distance remaining from the target position (in pulses)
extern long count; // Counter to track the number of pulses sent to the motor
/**
 * @brief Driver class for controlling the HSS86 stepper driver.
 * 
 * This class provides an interface for controlling the HSS86 stepper motor driver.
 * It allows setting the speed, direction, and movement of the stepper motor.
 */
class Driver2HSS86{
  public:
  /**
     * @brief Construct a new Driver2HSS86 object.
     * 
     * @param pulsesPerRevolution The number of pulses required for one full revolution of the stepper motor.
     */
  Driver2HSS86(long pulsesPerRevolution);
   /**
     * @brief Set the speed of the stepper motor in revolutions per minute (RPM).
     * 
     * @param revsPerMinute Desired speed in RPM.
     */
  void setSpeed(float revsPerMinute);
  /**
     * @brief Calculate the time period between pulses for a given RPM.
     * 
     * This static function calculates the time (in microseconds) between each pulse
     * based on the desired revolutions per minute and the number of pulses per revolution.
     * 
     * @param revsPerMinute The desired speed in RPM.
     * @param pulsesPerRev The number of pulses required for one revolution.
     * @return long The calculated time period between pulses in microseconds.
     */
  static long microSecsPerPulse(float revsPerMinute, long pulsesPerRev);
  /**
     * @brief Determine the appropriate prescaler for the timer based on the pulse period.
     * 
     * This static function selects a prescaler value that fits the given period into
     * the timer's resolution.
     * 
     * @param period Time period in microseconds between pulses.
     * @return int The selected prescaler value.
     */

  static int determinePrescaler(long period);
  /**
     * @brief Calculate the compare value for the timer based on the prescaler and period.
     * 
     * This static function calculates the compare match value for the timer to trigger
     * an interrupt at the desired pulse rate.
     * 
     * @param prescaler The prescaler value chosen for the timer.
     * @param period The time period in microseconds between pulses.
     * @return unsigned int The calculated compare value for the timer.
     */
  static unsigned int determineCompareVal(int prescaler, long period);
   /**
     * @brief Set the prescaler for Timer1.
     * 
     * This static function sets the prescaler for Timer1, which is used to control
     * the timing of the pulses sent to the stepper motor.
     * 
     * @param prescaler The prescaler value to be set for Timer1.
     */
  static void setPrescaler(int prescaler);
      /**
     * @brief Enable the stepper motor driver.
     * 
     * This function sets the enable pin high, allowing the stepper driver to control the motor.
     */
  void enable();
   /**
     * @brief Disable the stepper motor driver.
     * 
     * This function sets the enable pin low, disabling the stepper driver and stopping the motor.
     */
  void disable();
  /**
     * @brief Set the motor's rotation direction to clockwise.
     * 
     * This function sets the direction pin to rotate the stepper motor clockwise.
     */
  void setClockwise();
  /**
     * @brief Set the motor's rotation direction to counterclockwise.
     * 
     * This function sets the direction pin to rotate the stepper motor counterclockwise.
     */
  void setCounterClockwise();
   /**
     * @brief Move the stepper motor to a specified position.
     * 
     * This function moves the motor to a specific position based on the number of degrees
     * and speed specified in RPM. It calculates the required number of pulses to reach
     * the target position.
     * 
     * @param revsPerMinute The speed in RPM at which the motor should move.
     * @param degrees The target position in degrees to move the motor.
     */
  void setPos(float revsPerMinute, int degrees);
  
  private:
  long pulsesPerRev;// Number of pulses required for one full revolution of the motor
};
/**
 * @brief Interrupt Service Routine for Timer1 Compare Match A.
 * 
 * This ISR is triggered when Timer1 reaches the compare value. It generates
 * pulses for the stepper motor based on the timer configuration and motor speed.
 */
ISR(TIMER1_COMPA_vect);
/**
 * @brief Interrupt Service Routine for Timer1 Compare Match B.
 * 
 * This ISR is triggered when Timer1 reaches the compare value. It generates
 * pulses for the stepper motor based on the timer configuration and motor speed.
 */
ISR(TIMER1_COMPB_vect);

#endif
