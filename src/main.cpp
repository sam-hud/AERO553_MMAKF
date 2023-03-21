#include <Arduino.h>
#include <Wire.h>             // This library allows you to communicate with I2C devices.
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_SSD1306.h> // Hardware-specific library for SSD1306 displays
#include <HardwareSerial.h>   // Serial communication
#include <kalman.h>           // Kalman Filter library
#include "Motor.h"            // Header for motor class
#include "taskshare.h"        // Header for inter-task shared data
#include "taskqueue.h"        // Header for inter-task data queues
#include "encoder.h"          // Header for encoder class

// TODO: Check pins

// Motor pins
#define PWM_PIN 13
#define DIR_PIN 12
#define BRK_PIN 34

// Encoder pins
#define ENA_PIN 35
#define ENB_PIN 32

// Acoustic pins
#define REC_R 25
#define REC_L 33
#define TRANS 26

// Input pins
#define LIM_PIN 14
#define SLIDER_PIN 36
#define ACCEL_PIN 0

// OLED
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

/* Define Shares*/
Queue<float> motorSpeed(1, "Motor Speed");
Share<bool> limitSwitchPressed("Limit Switch Pressed");
Share<uint16_t> sliderPosition("Slider Position");
Share<float> actualMotorPosition("Actual Motor Position");
Share<float> KF1MotorPosition("KF1 Motor Position");
Share<float> KF2MotorPosition("KF2 Motor Position");
Share<float> KF3MotorPosition("KF3 Motor Position");
Share<float> MMAEMotorPosition("MMAE Motor Position");
Share<float> arduinoReading("Arduino Reading");
Share<float> accelerometerReading("Accelerometer Reading");

// Create each motor driver object
Motor motor(PWM_PIN, DIR_PIN, BRK_PIN);

// Create encoder object
encoder encoder(ENA_PIN, ENB_PIN);

// Create Kalman Filters
KalmanFilter KF1;
KalmanFilter KF2;
KalmanFilter KF3;

//********************************************************************************
// Task declarations
//********************************************************************************

//********************************************************************************
// Motor Task
void motorTask(void *p_params)
{
  while (true)
  {
    // TODO: set motor speed based on incoming queue data
    motor.setSpeed(127);
  }
}

void encoderTask(void *p_params)
{
  int32_t 
}

//********************************************************************************
// OLED display task
void displayTask(void *p_params)
{
  uint8_t displayState = 0; // Set start case to 0
  while (true)
  {
    // TODO: set task period based on incoming queue data
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);

    display.setCursor(0, 0);
    display.println("MMAKF");
    display.setTextSize(1);

    display.setCursor(10, 20);
    display.print("Actual:");
    display.setCursor(60, 20);
    display.print(actualMotorPosition.get());

    display.setCursor(10, 30);
    display.print("KF 1:");
    display.setCursor(60, 30);
    display.print(KF1MotorPosition.get());

    display.setCursor(10, 40);
    display.print("KF 2:");
    display.setCursor(60, 40);
    display.print(KF2MotorPosition.get());

    display.setCursor(10, 50);
    display.print("KF 3:");
    display.setCursor(60, 50);
    display.print(KF3MotorPosition.get());

    display.setCursor(10, 60);
    display.print("MMAKF: ");
    display.setCursor(60, 60);
    display.print(MMAEMotorPosition.get());

    display.display();
    vTaskDelay(100); // Task period
  }
}
//********************************************************************************
// Input slider task
void controlInputTask(void *p_params)
{
  while (true)
  {
    // TODO: read slider input and set shared variable
    uint16_t sliderValue = analogRead(SLIDER_PIN);
    sliderPosition.put(sliderValue);
    vTaskDelay(100); // Task period
  }
}
//********************************************************************************
// Accelerometer reading task
void accelerometerTask(void *p_params)
{
  uint8_t accelState = 0; // Set start case to 0
  while (true)
  {
    // TODO: read accelerometer and set shared variable
    float accelValue = analogRead(ACCEL_PIN);
    accelerometerReading.put(accelValue);
    vTaskDelay(100); // Task period
  }
}
//********************************************************************************
// Arduino communication task
// void arduinoTask(void *p_params)
// {
//   while (true)
//   {
//     while (Arduino.available())
//     {
//       String arduinoRead = Arduino.readString();
//       arduinoReading.put(arduinoRead.toFloat());
//       // Task period controlled by Arduino reading
//     }
//   }
// }
//********************************************************************************
// Limit switch task
void limitSwitchTask(void *p_params)
{
  while (true)
  {
    bool limitSwitchStatus = digitalRead(LIM_PIN);
    if (limitSwitchStatus == HIGH) // TODO: check if this is correct
    {
      limitSwitchPressed.put(true);
    }
    else
    {
      limitSwitchPressed.put(false);
    }
    vTaskDelay(100); // Task period
  }
}

//********************************************************************************
// Kalman filter tasks
void KF1Task(void *p_params)
{
  // Kalman filter setup
  unsigned long timer = 0;
  KF1.init(2);
  KF1.setProcessNoise(0.1, 0.01);
  KF1.setMeasurementNoise(0.1);

  while (true)
  {
    float measurement0 = arduinoReading.get();
    float measurement1 = accelerometerReading.get();

    // Delta time : time since last prediction
    float dt = (millis() - timer) / 1000.f;
    timer = millis();

    // Kalman filter steps ( TODO: need to add in accelerometer readings)
    KF1.predict(dt);
    int x = KF1.get();
    KF1.correct(measurement1);
    vTaskDelay(100); // Task period
  }
}

void KF2Task(void *p_params)
{
  // Kalman filter setup
  unsigned long timer = 0;
  KF2.init(2);
  KF2.setProcessNoise(0.1, 0.01);
  KF2.setMeasurementNoise(0.1);

  while (true)
  {
    float measurement0 = arduinoReading.get();
    float measurement1 = accelerometerReading.get();

    // Delta time : time since last prediction
    float dt = (millis() - timer) / 1000.f;
    timer = millis();

    // Kalman filter steps ( TODO: need to add in accelerometer readings)
    KF2.predict(dt);
    int x = KF3.get();
    KF2.correct(measurement1);
    vTaskDelay(100); // Task period
  }
}

void KF3Task(void *p_params)
{
  // Kalman filter setup
  unsigned long timer = 0;
  KF3.init(2);
  KF3.setProcessNoise(0.1, 0.01);
  KF3.setMeasurementNoise(0.1);

  while (true)
  {
    float measurement0 = arduinoReading.get();
    float measurement1 = accelerometerReading.get();

    // Delta time : time since last prediction
    float dt = (millis() - timer) / 1000.f;
    timer = millis();

    // Kalman filter steps ( TODO: need to add in accelerometer readings)
    KF3.predict(dt);
    int x = KF3.get();
    KF3.correct(measurement1);
    vTaskDelay(100); // Task period
  }
}
void MMAETask(void *p_params)
{
  while (true)
  {
    // Compares each KF to actual measurement and weights them based on their error

    // Calculate errors
    float KF1Error = abs(KF1MotorPosition.get() - actualMotorPosition.get());
    float KF2Error = abs(KF2MotorPosition.get() - actualMotorPosition.get());
    float KF3Error = abs(KF3MotorPosition.get() - actualMotorPosition.get());

    float totalError = KF1Error + KF2Error + KF3Error;

    // Calculate weights
    float KF1Weight = KF1Error / totalError;
    float KF2Weight = KF2Error / totalError;
    float KF3Weight = KF3Error / totalError;

    // Calculate MMAE output
    float MMAEOutput = (KF1MotorPosition.get() * KF1Weight) + (KF2MotorPosition.get() * KF2Weight) + (KF3MotorPosition.get() * KF3Weight);
    MMAEMotorPosition.put(MMAEOutput);

    vTaskDelay(100); // Task period
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
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRK_PIN, OUTPUT);
  pinMode(ENA_PIN, INPUT_PULLUP);
  pinMode(ENB_PIN, INPUT_PULLUP);
  pinMode(LIM_PIN, INPUT);
  pinMode(SLIDER_PIN, INPUT);

  // Reset OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  // Initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  {
    Serial.println(F("OLED not found"));
    while (1)
      ;
  }

  // Start FreeRTOS tasks
  xTaskCreate(displayTask, "Display Task", 10000, NULL, 2, NULL);
  xTaskCreate(controlInputTask, "Control input Task", 4096, NULL, 3, NULL);
  xTaskCreate(accelerometerTask, "Accelerometer Task", 4096, NULL, 3, NULL);
  xTaskCreate(motorTask, "Motor Task", 4096, NULL, 3, NULL);
  // xTaskCreate(arduinoTask, "Arduino Communication Task", 4096, NULL, 3, NULL);
  xTaskCreate(limitSwitchTask, "Limit Switch Task", 4096, NULL, 3, NULL);
  // xTaskCreate(KF1Task, "KF1 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(KF2Task, "KF2 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(KF3Task, "KF3 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(MMAETask, "MMAE Task", 10000, NULL, 3, NULL);
}

/**
 * @brief Main loop that does nothing (ensures FreeRTOS does not crash)
 *
 */
void loop()
{
  delay(60000);
};