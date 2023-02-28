/**
 * @file LimitSwitchTask.cpp
 * @author Dylan Ruiz
 * @brief Provides the FSM for checking the limit switches
 * @version 1.0
 * @date 2022-11-23
 */

#include <Arduino.h>
#include "shares.h"
#include "tasks/LimitSwitchTask.h"

/**
 * @brief Construct a new Limit Switch Task
 *
 * @param XLIM_PIN X limit switch pin
 * @param YLIM_PIN Y limit switch pin
 */
LimitSwitchTask::LimitSwitchTask(uint8_t XLIM_PIN, uint8_t YLIM_PIN)
{
    int state = 0; // Start state
    // Set pins
    this->XLIM_PIN = XLIM_PIN;
    this->YLIM_PIN = YLIM_PIN;

    // Ensures task does not start until start signal is received
    startLimitx.put(false);
    startLimity.put(false);
}
/**
 * @brief Run the FSM for the limit switches
 *
 */
void LimitSwitchTask::run() // Method for FSM
{
    switch (state)
    {
    case 0: // Wait for start signal
    {
        if (startLimitx.get())
        {
            state = 1; // Go to check x state
            startLimitx.put(false);
        }
        if (startLimity.get())
        {
            state = 2; // Go to check y state
            startLimity.put(false);
        }
        break;
    }

    case 1: // Check x limit switch
    {
        if (digitalRead(XLIM_PIN) == LOW)
        {
            // Stop motors
            stopMotor1.put(true);
            stopMotor2.put(true);
            state = 0;
        }
        break;
    }
    case 2: // Check y limit switch
    {
        if (digitalRead(YLIM_PIN) == LOW)
        {
            // Stop motors
            stopMotor1.put(true);
            stopMotor2.put(true);
            state = 0;
        }
        break;
    }
    }
}