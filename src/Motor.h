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
    /** @brief Variable for Pin1
     */
    uint8_t PIN_1; ///< Motor Pin1
    /** @brief Variable for Pin2
     */
    uint8_t PIN_2; ///< Motor Pin2
public:
    /**
     * @brief Construct a new Motor object
     *
     * @param PIN_1 motor pin 1
     * @param PIN_2 motor pin 2
     */
    Motor(uint8_t PIN_1, uint8_t PIN_2);
    /** @brief Method to set motor speed
     *   @params PWM to set the motor speed
     */
    void SetSpeed(uint16_t PWM);
};

#endif
