/** @file IMU.cpp
 *  @brief This file contains data relevant to interfacing with the MPU 6050 module
 *         most content pulled from https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/
 *  @author Michael Schoeffler 
 *  @author Nolan Clapp
 *  @date 2017 Original Schoeffler 
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use Clapp
 *  
 */

#include <Arduino.h>
#include <PrintStream.h>
#include "IMU.h"
// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
// https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void IMU_setup() {

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register Ridgley
  //Wire.write(0x19); //Amazon
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  vTaskDelay(100);
  
}
void IMU_get_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40] Ridgley
  //Wire.write(0x1E); // amazon

  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, 1); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable

  accelerometer_x = (((uint16_t) Wire.read())<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = (((uint16_t) Wire.read())<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = (((uint16_t) Wire.read())<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)

  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  float pitch = 180 * atan (accelerometer_x/sqrt(accelerometer_y*accelerometer_y + accelerometer_z*accelerometer_z))/M_PI;
  float roll = 180 * atan (accelerometer_y/sqrt(accelerometer_x*accelerometer_x + accelerometer_z*accelerometer_z))/M_PI;
  
  float yaw = 180 * atan (accelerometer_z/sqrt(accelerometer_x*accelerometer_x + accelerometer_z*accelerometer_z))/M_PI;
 
 //Serial<< setprecision(3) <<"[R: "<< roll << "  P: "<< pitch<< "  Y: "<< yaw<<"]"<<endl;
  // print out data
  Serial << "Accelerometer: ["<< "X="<<convert_int16_to_str(accelerometer_x)<< "  "<<
                                "Y="<<convert_int16_to_str(accelerometer_y)<< "  "<<
                                "Z="<<convert_int16_to_str(accelerometer_z)<<" ]";

  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]

  Serial << "  Gyro ["<< "X="<<convert_int16_to_str(gyro_x)<< "  "<<
                         "Y="<<convert_int16_to_str(gyro_y)<< "  "<<
                         "Z="<<convert_int16_to_str(gyro_z)<< " ]"<< endl;

}