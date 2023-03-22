/** @file IMU.h
 *  @brief Headder file for IMU
    @author Clapp
    @author Schoeffler
 *  @date 2017 Original Schoeffler 
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use Clapp
 */

#ifndef IMU_h
#define IMU_h

#include <Arduino.h> 
#include "PrintStream.h"



/** @brief Function to set up IMU
 */
void IMU_setup();
/** @brief Function to get IMU data
 */
void IMU_get_data();


#endif
