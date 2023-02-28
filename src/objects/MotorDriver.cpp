/**
 * @file MotorDriver.h
 * @author Dylan Ruiz
 * @brief Motor control specific to the TMC2208 stepper driver
 * @version 1.0
 * @date 2022-10-29
 *
 */

#include <Arduino.h>
#include "objects/MotorDriver.h"
#include "shares.h"

/**
 * @brief Construct a new Motor object
 *
 * @param ENABLE_PIN Enable pin for the motor
 * @param STEP_PIN Step pin for the motor
 * @param DIRECTION_PIN Direction pin for the motor
 */
Motor::Motor(uint8_t ENABLE_PIN, uint8_t STEP_PIN, uint8_t DIRECTION_PIN)
{
    this->ENABLE_PIN = ENABLE_PIN;
    this->STEP_PIN = STEP_PIN;
    this->DIRECTION_PIN = DIRECTION_PIN;

    // Setup pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, LOW); // Enable motor
}
/**
 * @brief Default constructor for a new Motor object (needed for motor task class dependency)
 *
 */
Motor::Motor()
{
}
/**
 * @brief Start Motor at a fixed maximum angular velocity
 * @param Dir indicates which direction the motor runs(Dir = 1 or Dir = -1)
 * @param Steps indicates number of steps the motor should run for
 * @param stopFlag A share bool variable indicating whether the motor should stop prematurely.
 */
void Motor::startMax(int8_t Dir, uint16_t Steps, Share<bool> &stopFlag)
{
    stopFlag.put(false); // Start motor

    if (Dir == 1)
    {
        digitalWrite(DIRECTION_PIN, HIGH);
    }
    else if (Dir == -1)
    {
        digitalWrite(DIRECTION_PIN, LOW);
    }
    int i = 0;
    for (i = 0; i < 2 * Steps; i++)
    {

        if (stopFlag.get() == true)
        {
            break;
        }
        digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
        delay(1);
    }
    stopFlag.put(true); // Stop motor
}
void Motor::start(float velocity, uint16_t Steps, Share<bool> &stopFlag)
{
    stopFlag.put(false); // Start motor

    // Check and set motor direction
    if (velocity > 0)
    {
        digitalWrite(DIRECTION_PIN, HIGH);
    }
    else if (velocity <= 0)
    {
        digitalWrite(DIRECTION_PIN, LOW);
    }

    uint32_t delay_time;

    // Ensure no division by zero
    if (velocity = 0)
    {
        delay_time = 1;
    }
    else
    {
        delay_time = 1000000 / abs(velocity);
    }

    // Iterate through steps until the motor should stop
    for (int i = 0; i < 2 * Steps; i++)
    {
        if (stopFlag.get() == true) // Check if the motor should stop
        {
            break; // Break out of loop to stop motor
        }
        digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); // Alternate step pin state
        delayMicroseconds(delay_time);
    }
    stopFlag.put(true); // Stop motor
}
