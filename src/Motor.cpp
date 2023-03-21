/** @file Motor.cpp
 *  @brief Creates a motor class to interface with a brushless DC motor
 *  @details It works by seting motor pins as inputs and passing a PWM signal to
 *  one of the pins and sets the other as high per the motor driver data sheet.
 *  @author Nolan Clapp
 *  @date 2022-Oct-26 Original file by Clapp
 *
 */

#include <Arduino.h>
#include "Motor.h"
// #include <PrintStream.h>

/** @brief   Constructor which creates a motor object.
 */
Motor::Motor(uint8_t PWM_PIN, uint8_t DIR_PIN, uint8_t BRK_PIN)
{
  this->PWM_PIN = PWM_PIN;
  this->DIR_PIN = DIR_PIN;
  this->BRK_PIN = BRK_PIN;
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRK_PIN, OUTPUT);
}
/** @brief   Method sets the motor speed (-255 to +255)
 *  @details This method sets the motor speed by passing in a PWM
 *           signal and setting one motor pin to high
 *  @params PWM The motor speed (0=Off; 255=100% FWD, -255=100% REV)
 */
void Motor::setSpeed(int16_t signedSpeed)
{
  digitalWrite(Motor::BRK_PIN, 1);

  if (signedSpeed >= 0)
  {
    if (signedSpeed > 255)
    {
      signedSpeed = 255;
    }
    digitalWrite(Motor::DIR_PIN, 1);
    analogWrite(Motor::PWM_PIN, signedSpeed);
  }
  else
  {
    if (signedSpeed < -255)
    {
      signedSpeed = -255;
    }
    digitalWrite(Motor::DIR_PIN, 0);
    analogWrite(Motor::PWM_PIN, -signedSpeed);
  }
}
