/** @file Motor.h
 *  This is the .h file associated with our motor class
 *  @author Nolan Clapp
 *  @date 2022-Oct-26 Original file by Clapp
 */

#ifndef Motor_h
#define Motor_h

#include <Arduino.h> 
#include "PrintStream.h"

/** @brief   Class which deals with motor interaction
 */
class Motor
{
protected:

public:
    /** @brief Variable for Pin1
    */
    uint16_t PIN_1;           ///< Motor Pin1
    /** @brief Variable for Pin2
    */
    uint16_t PIN_2;           ///< Motor Pin2
    /** @brief Motor init
    */
    Motor (void);
    /** @brief Method to set motor speed
    *   @params PWM to set the motor speed
    */
    void SetSpeed(uint16_t PWM);
    /** @brief Method to get the current draw of motor driver
    *   @return Current returns the motor current
    */
    float getcurrent(void);
    

};

#endif
