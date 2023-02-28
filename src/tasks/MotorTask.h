/**
 * @file MotorTask.h
 * @author Dylan Ruiz
 * @brief Provides an FSM for a motor
 * @version 1.0
 * @date 2022-11-23
 */

#ifndef _MOTOR_TASK_H
#define _MOTOR_TASK_H

#include <Arduino.h>
#include "taskqueue.h"
#include "taskshare.h"
#include "objects/MotorDriver.h"

class MotorTask
{
private:
    Motor motor;
    uint8_t state;
    Share<bool> stopMotor;
    Share<int8_t> direction;
    Share<float> velocity;
    Share<uint16_t> steps;
    Share<uint8_t> startMotor;

public:
    MotorTask(Motor motor, Share<bool> &stopMotor, Share<int8_t> &direction, Share<float> &velocity, Share<uint16_t> &steps, Share<uint8_t> &startMotor);
    void run();
};

#endif // _MOTOR_TASK_H