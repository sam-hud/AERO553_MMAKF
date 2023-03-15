/**
 * @file main.cpp
 * @author Sam Hudson, Dylan Ruiz, Scott Dunn
 * @brief Main file for controlling the ChessBot
 * @version 1.0
 * @date 2022-12-06
 *
 */
#include <Arduino.h>
#include <Wire.h>             // This library allows you to communicate with I2C devices.
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_SSD1306.h> // Hardware-specific library for SSD1306 displays
#include <HardwareSerial.h>   // Serial communication
#include "Motor.h"            // Header for motor class
#include "taskshare.h"        // Header for inter-task shared data
#include "taskqueue.h"        // Header for inter-task data queues
#include "shares.h"           // Header for shares used in this project

// Motor pins
#define MOTOR_1 12
#define MOTOR_2 13

// Input pins
#define LIM_PIN 2
#define SLIDER_PIN 3

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
Motor motor(MOTOR_1, MOTOR_2);

//********************************************************************************
// Task declarations
//********************************************************************************

//********************************************************************************
// Main FSM
void mainController(void *p_params)
{
  uint8_t mainState = 0; // Set start case to 0
  while (true)
  {
    switch (mainState)
    {
    case 0: // Homing sequence
    {

      break;
    }
    case 1: // Check if motors have stopped
    {

      break;
    }
      vTaskDelay(100); // Task period
    }
  }
}
//********************************************************************************
// OLED display task
void displayTask(void *p_params)
{
  uint8_t displayState = 0; // Set start case to 0
  while (true)
  {
    switch (displayState)
    {
    case 0: // Homing sequence
    {

      break;
    }
    case 1: // Check if motors have stopped
    {

      break;
    }
      vTaskDelay(100); // Task period
    }
  }
}
//********************************************************************************
// Input slider task
void controlInputTask(void *p_params)
{
  uint8_t controlInputState = 0; // Set start case to 0
  while (true)
  {
    switch (controlInputState)
    {
    case 0: // Homing sequence
    {

      break;
    }
    case 1: // Check if motors have stopped
    {

      break;
    }
      vTaskDelay(100); // Task period
    }
  }
}
//********************************************************************************
// Accelerometer reading task
void accelerometerTask(void *p_params)
{
  uint8_t accelState = 0; // Set start case to 0
  while (true)
  {
    switch (accelState)
    {
    case 0: // Homing sequence
    {

      break;
    }
    case 1: // Check if motors have stopped
    {

      break;
    }
      vTaskDelay(100); // Task period
    }
  }
}
//********************************************************************************
// Arduino communication task
void arduinoTask(void *p_params)
{
  uint8_t arduinoState = 0; // Set start case to 0
  while (true)
  {
    switch (arduinoState)
    {
    case 0: // Homing sequence
    {

      break;
    }
    case 1: // Check if motors have stopped
    {

      break;
    }
      vTaskDelay(100); // Task period
    }
  }
}
//********************************************************************************
// Limit switch task
void limitSwitchTask(void *p_params)
{
  uint8_t limitState = 0; // Set start case to 0
  while (true)
  {
    switch (limitState)
    {
    case 0: // Check if limit switch is pressed
    {
      if (digitalRead(LIM_PIN) == HIGH)
      {
        limitState = 1;
      }
      break;
    }
      {

        vTaskDelay(100); // Task period
      }
    }
  }
}

//********************************************************************************
// End of Task declarations
//********************************************************************************


//********************************************************************************
// Setup FreeRTOS tasks and start scheduler
//********************************************************************************
void setup()
{
  Serial.begin(115200); // Begin serial monitor
  while (!Serial)
  {
  } // Wait for port to be ready before continuing

  // Setup pins
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(LIM_PIN, INPUT);
  pinMode(SLIDER_PIN, INPUT);

  // Start FreeRTOS tasks
  xTaskCreate(mainController, "Motor 1 Task", 10000, NULL, 3, NULL);
  xTaskCreate(displayTask, "Display Task", 10000, NULL, 2, NULL);
  xTaskCreate(controlInputTask, "Control input Task", 4096, NULL, 3, NULL);
  xTaskCreate(accelerometerTask, "Accelerometer Task", 4096, NULL, 3, NULL);
  xTaskCreate(arduinoTask, "Arduino Communication Task", 4096, NULL, 3, NULL);
  xTaskCreate(limitSwitchTask, "Limit Switch Task", 4096, NULL, 3, NULL);
}

/**
 * @brief Main loop that does nothing (ensures FreeRTOS does not crash)
 *
 */
void loop()
{
  delay(60000);
};