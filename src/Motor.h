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
    /** @brief Variable for PWM pin
     */
    uint8_t PWM_PIN; ///< Motor PWM/Speed pin
    /** @brief Variable for DIR pin
     */
    uint8_t DIR_PIN; ///< Motor DIR pin
    /** @brief Variable for Brake pin
     */
    uint8_t BRK_PIN; ///< Motor BRK pin
public:
    /**
     * @brief Construct a new Motor object
     *
     * @param PWM_PIN motor PWM
     * @param DIR_PIN motor DIR
     * @param BRK_PIN motor BRK
     */
    Motor(uint8_t PWM_PIN, uint8_t DIR_PIN, uint8_t BRK_PIN);
    /** @brief Method to set motor speed
     *   @params PWM to set the motor speed
     */
    void setSpeed(int16_t SignedSpeed);
};

#endif
