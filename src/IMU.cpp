/** @file IMU.cpp
 *  @brief This file contains data relevant to interfacing with the MPU 6050 module
 *         most content pulled from https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/
 *  @author Michael Schoeffler
 *  @author Nolan Clapp
 *  @author Joe Lyons
 *  @date 2017 Original Schoeffler
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use Clapp/Lyons
 *
 */

#include <Arduino.h>
#include "IMU.h"
// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
// https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
// int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
// int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char *convert_int16_to_str(int16_t i)
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setupIMU()
{

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                 // PWR_MGMT_1 register Ridgley
  // Wire.write(0x19); //Amazon
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  vTaskDelay(100);
}

float getIMU_x()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40] Ridgley

  Wire.endTransmission(false);          // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, 1); // request a total of 7*2=14 registers

  accelerometer_x = (((uint16_t)Wire.read()) << 8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)

  return accelerometer_x;
}

float getIMU_y()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40] Ridgley

  Wire.endTransmission(false);          // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, 1); // request a total of 7*2=14 registers

  accelerometer_y = (((uint16_t)Wire.read()) << 8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)

  return accelerometer_y;
}

float getIMU_z()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40] Ridgley

  Wire.endTransmission(false);          // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, 1); // request a total of 7*2=14 registers

  accelerometer_z = (((uint16_t)Wire.read()) << 8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  return accelerometer_z;
}