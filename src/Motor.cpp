/** @file Motor.cpp
 *  @brief Creates a motor class to interface with a brushless DC motor
 *  @details It works by seting motor pins as inputs and passing a PWM signal to
 *  one of the pins and sets the other as high per the motor driver data sheet.
 *  @author Nolan Clapp
 *  @date 2022-Oct-26 Original file by Clapp
 *
 */

#include <Arduino.h>
#include <PrintStream.h>
#include "Motor.h"

/** @brief   Constructor which creates a motor object.
 */
Motor::Motor(uint8_t PIN_1, uint8_t PIN_2)
{
  this->PIN_1 = PIN_1;
  this->PIN_2 = PIN_2;
  pinMode(PIN_1, INPUT);
  pinMode(PIN_2, INPUT);
}
/** @brief   Method sets the motor speed (0-255)
 *  @details This method sets the motor speed by passing in a PWM
 *           signal and setting one motor pin to high
 *  @params PWM The motor speed (0=Off; 255=100%)
 */
void Motor::setSpeed(uint16_t PWM)
{
  analogWrite(PIN_1, PWM);
  digitalWrite(PIN_2, HIGH);
}
