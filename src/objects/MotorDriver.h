/**
 * @file MotorDriver.h
 * @author Dylan Ruiz
 * @brief Motor control specific to the TMC2208 stepper driver
 * @version 1.0
 * @date 2022-10-29
 *
 */

#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <Arduino.h>
#include "taskshare.h"

class Motor

{
private:
    uint8_t ENABLE_PIN;
    uint8_t STEP_PIN;
    uint8_t DIRECTION_PIN;

public:
    Motor(uint8_t enable_pin, uint8_t step_pin, uint8_t direction_pin);
    Motor(); // Default constructor

    void startMax(int8_t dir, uint16_t steps, Share<bool> &stopFlag);
    void start(float velocity, uint16_t steps, Share<bool> &stopFlag);
};

#endif // _MOTOR_DRIVER_H_
