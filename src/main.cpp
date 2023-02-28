/**
 * @file main.cpp
 * @author Sam Hudson, Dylan Ruiz, Scott Dunn
 * @brief Main file for controlling the ChessBot
 * @version 1.0
 * @date 2022-12-06
 *
 */
#include <Arduino.h>
#include "taskshare.h" // Header for inter-task shared data
#include "taskqueue.h" // Header for inter-task data queues
#include "shares.h"    // Header for shares used in this project
#include "objects/MotorDriver.h"
#include "tasks/ControllerTask.h"
#include "tasks/MotorTask.h"
#include "tasks/LimitSwitchTask.h"

// Motor 1 pins
#define EN_PIN_1 14   // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_1 33 // Step on rising edge
#define DIR_PIN_1 32

// Motor 2 pins
#define EN_PIN_2 13   // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_2 26 // Step on rising edge
#define DIR_PIN_2 25

// Limit switch pins
#define XLIM_PIN 2
#define YLIM_PIN 15

// Solenoid pin
#define SOLENOID_PIN 4

// Sensor pin
#define SENSOR_PIN 39

/* Define Shares*/
Share<bool> stopMotor1("Stop Motor 1");
Share<bool> stopMotor2("Stop  Motor 2");
Share<bool> beginMove("Begin Move");
Queue<float> directionsQueue(5, "Directions Queue");
Share<uint16_t> steps1("No. of steps for Motor 1");
Share<uint16_t> steps2("No. of steps for Motor 2");
Share<float> velocity1("Steps/sec for Motor 1");
Share<float> velocity2("Steps/sec for Motor 2");
Share<int8_t> dirMotor1("Motor 1 Direction");
Share<int8_t> dirMotor2("Motor 2 Direction");
Share<uint8_t> startMotor1("Start Motor 1");
Share<uint8_t> startMotor2("Start Motor 2");
Share<bool> moveComplete("Move Complete");
Share<bool> startLimitx("Start Limit X");
Share<bool> startLimity("Start Limit Y");

// Create each motor driver object
Motor motor1(EN_PIN_1, STEP_PIN_1, DIR_PIN_1);
Motor motor2(EN_PIN_2, STEP_PIN_2, DIR_PIN_2);

// Create motor task objects using motor driver objects
MotorTask motorTask1(motor1, stopMotor1, dirMotor1, velocity1, steps1, startMotor1);
MotorTask motorTask2(motor2, stopMotor2, dirMotor2, velocity2, steps2, startMotor2);

// Create Limit Switch task object
LimitSwitchTask limitTask(XLIM_PIN, YLIM_PIN);

// Create main controller
Controller mainController(SOLENOID_PIN, SENSOR_PIN);

/* --- Define tasks for FreeRTOS --- */

/**
 * @brief Task for controlling motor 1
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void defMotorTask1(void *p_params)
{
  while (true)
  {
    motorTask1.run();
    vTaskDelay(20); // Task period
  }
}

/**
 * @brief Task for controlling motor 2
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void defMotorTask2(void *p_params)
{
  while (true)
  {
    motorTask2.run();
    vTaskDelay(20); // Task period
  }
}

/**
 * @brief Task for main controller
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void defControllerTask(void *p_params)
{
  while (true)
  {
    mainController.run();
    vTaskDelay(100); // Task period
  }
}

/**
 * @brief Task for checking limit switches
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void defLimitTask(void *p_params)
{
  while (true)
  {
    limitTask.run();
    vTaskDelay(20); // Task period
  }
}

/* --- End of task definitions for FreeRTOS tasks --- */

/* --- Setup and begin multitasking --- */

/**
 * @brief Main program that sets up FreeRTOS tasks and starts the scheduler
 *
 */
void setup()
{
  Serial.begin(115200); // Begin serial monitor
  while (!Serial)
  {
  } // Wait for port to be ready before continuing

  // Setup pins
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(XLIM_PIN, INPUT);
  pinMode(YLIM_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  // Start FreeRTOS tasks
  xTaskCreate(defMotorTask1, "Motor 1 Task", 10000, NULL, 3, NULL);
  xTaskCreate(defMotorTask2, "Motor 2 Task", 10000, NULL, 3, NULL);
  xTaskCreate(defControllerTask, "Controller Task", 10000, NULL, 2, NULL);
  xTaskCreate(defLimitTask, "Limit Task", 4096, NULL, 3, NULL);
}

/**
 * @brief Main loop that does nothing (ensures FreeRTOS does not crash)
 *
 */
void loop()
{
  delay(60000);
};