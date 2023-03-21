/** @file IMU.h
 *  @brief Header file for IMU
    @author Clapp
    @author Lyons
    @author Schoeffler
 *  @date 2017 Original Schoeffler
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use Clapp/Lyons
 */

#ifndef IMU_h
#define IMU_h

#include <Arduino.h>


/** @brief Function to set up IMU
 */
void setupIMU();
/** @brief Functions to get IMU data
 */
float getIMU_x();
float getIMU_y();
float getIMU_z();

#endif
