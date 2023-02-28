/**
 * @file LimitSwitchTask.h
 * @author Dylan Ruiz
 * @brief Provides the FSM for checking the limit switches
 * @version 1.0
 * @date 2022-11-23
 */

#ifndef _LIMIT_SWITCH_TASK_H
#define _LIMIT_SWITCH_TASK_H

#include <Arduino.h>
#include "taskqueue.h"
#include "taskshare.h"
#include "objects/MotorDriver.h"

class LimitSwitchTask
{
private:
    uint8_t state;
    uint8_t XLIM_PIN;
    uint8_t YLIM_PIN;

public:
    LimitSwitchTask(uint8_t XLIM_PIN, uint8_t YLIM_PIN);
    void run();
 
};
#endif //_LIMIT_SWITCH_TASK_H