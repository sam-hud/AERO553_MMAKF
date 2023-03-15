/**
 * @file main.cpp
 * @author Sam Hudson, Dylan Ruiz, Scott Dunn
 * @brief Main file for controlling the ChessBot
 * @version 1.0
 * @date 2022-12-06
 *
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include "Motor.h"
#include "taskshare.h" // Header for inter-task shared data
#include "taskqueue.h" // Header for inter-task data queues
#include "shares.h"    // Header for shares used in this project

// Motor pins
#define EN_PIN_1 14   // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_1 33 // Step on rising edge
#define DIR_PIN_1 32

// OLED
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

/* Define Shares*/
// Share<bool> stopMotor1("Stop Motor 1");
// Share<bool> stopMotor2("Stop  Motor 2");
// Share<bool> beginMove("Begin Move");
// Queue<float> directionsQueue(5, "Directions Queue");
// Share<uint16_t> steps1("No. of steps for Motor 1");
// Share<uint16_t> steps2("No. of steps for Motor 2");
// Share<float> velocity1("Steps/sec for Motor 1");
// Share<float> velocity2("Steps/sec for Motor 2");
// Share<int8_t> dirMotor1("Motor 1 Direction");
// Share<int8_t> dirMotor2("Motor 2 Direction");
// Share<uint8_t> startMotor1("Start Motor 1");
// Share<uint8_t> startMotor2("Start Motor 2");
// Share<bool> moveComplete("Move Complete");
// Share<bool> startLimitx("Start Limit X");
// Share<bool> startLimity("Start Limit Y");

// Create each motor driver object
Motor motor1(EN_PIN_1, STEP_PIN_1, DIR_PIN_1);

// Motor, screen, slider, accelerometer, arduino,

/**
 * @brief Task for main controller
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void mainController(void *p_params)
{
  while (true)
  {
    switch (case)
    {
      switch (state)
      {
      case 0: // Homing sequence
      {

        break;
      }
      case 1: // Check if motors have stopped
      {

        break;
      }
      }
      vTaskDelay(100); // Task period
    }
  }
}

/**
 * @brief Task for checking limit switches
 *
 * @param p_params void pointer for FreeRTOS setup
 */
void limitSwitchTask(void *p_params)
{
  while (true)
  {
    // check limit switches

    vTaskDelay(100); // Task period
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
  pinMode(LIM_PIN, INPUT);
  pinMode(SLIDER_PIN, INPUT);

  // Start FreeRTOS tasks
  xTaskCreate(defMotorTask, "Motor 1 Task", 10000, NULL, 3, NULL);
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