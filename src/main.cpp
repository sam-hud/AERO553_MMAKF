#include <Arduino.h>
#include <Wire.h>                        // This library allows you to communicate with I2C devices.
#include <Adafruit_GFX.h>                // Core graphics library
#include <Adafruit_SSD1306.h>            // Hardware-specific library for SSD1306 displays
#include <HardwareSerial.h>              // Serial communication
#include <kalman.h>                      // Kalman Filter library
#include "Motor.h"                       // Header for motor class
#include "ME507-Support/src/taskshare.h" // Header for inter-task shared data
#include "ME507-Support/src/taskqueue.h" // Header for inter-task data queues
#include <ESP32Encoder.h>
#include "IMU.h"
#include <StateSpaceControl.h>

// TODO: Check pins

// Motor pins
#define PWM_PIN 13
#define DIR_PIN 12
#define BRK_PIN 15

// Encoder pins
#define ENA_PIN 2
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
Share<float> motorSpeed("Motor Speed");
Share<bool> limitSwitchPressed("Limit Switch Pressed");
Share<int16_t> sliderPosition("Slider Position");
Share<float> actualMotorPosition("Actual Motor Position");
Share<float> KF1MotorPosition("KF1 Motor Position");
Share<float> KF2MotorPosition("KF2 Motor Position");
Share<float> KF3MotorPosition("KF3 Motor Position");
Share<float> MMAEMotorPosition("MMAE Motor Position");
Share<float> arduinoReading("Arduino Reading");
Share<float> accelerometerReading("Accelerometer Reading");
Share<int16_t> encPos("Encoder position");
Share<unsigned long> rightTime("Right Acoustic Time");
Share<unsigned long> leftTime("Left Acoustic Time");
Share<float> acousticPosition("Acoustic Position");

// Create each motor driver object
Motor motor(PWM_PIN, DIR_PIN, BRK_PIN);

// Create encoder
ESP32Encoder encoder;

// Create Kalman Filters
KalmanFilter KF1;
KalmanFilter KF2;
KalmanFilter KF3;

// Create state-space controller
MotorPositionModel model(0.0000011502, 0.01, 0.4982, 4.8, 0.05);
StateSpaceController<3, 1> ssController(model);
Matrix<3> y;
const float dt = 0.01;

void rightInt()
{
  rightTime.put(micros());
}

void leftInt()
{
  leftTime.put(micros());
}

//********************************************************************************
// Task declarations
//********************************************************************************

//********************************************************************************
// Motor Task
void motorTask(void *p_params)
{
  while (limitSwitchPressed.get() == false)
  {
    motor.setSpeed(-120);
    vTaskDelay(100);
  }
  motor.setSpeed(150);
  vTaskDelay(250);
  motor.setSpeed(0);
  while (true)
  {
    motor.setSpeed(motorSpeed.get());
    vTaskDelay(10); // Task period
  }
}
//********************************************************************************
// Encoder Task
void encoderTask(void *p_params)
{
  float temp_pos = 0.0;
  while (true)
  {
    temp_pos = encoder.getCount();                   // measure position
    temp_pos = temp_pos * 8 * 100 / 12.59 / 1632.67; // convert to mm, linear
    actualMotorPosition.put(temp_pos);
    // Serial.print("Encoder position: ");
    // Serial.println(temp_pos);
    vTaskDelay(25); // Task period
  }
}
//********************************************************************************
// Motor positioner task
void ssControllerTask(void *p_params)
{
  ssController.K = {22.3607, 0.00039252, 1.9114};
  ssController.L = {10.0347, 0.3480, 0.0065};
  ssController.initialise();
  float actual_motor_pos = 0.0;
  float desired_motor_pos = 0.0;
  uint8_t dt = 50;
  float gain = 0.0;
  while (true)
  {
    desired_motor_pos = sliderPosition.get();
    ssController.r(0) = desired_motor_pos;
    actual_motor_pos = actualMotorPosition.get();
    ssController.update(actual_motor_pos, (dt / 1000));
    gain = ssController.u(0);
    gain = gain * 255 / 50 / 12;
    motorSpeed.put(gain);
    Serial.println(gain);
    vTaskDelay(dt);
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
//********************************************************************************
// OLED display task
void displayTask(void *p_params)
{
  while (true)
  {
    /*
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
    */
    /* Serial monitor output */
    Serial.println("Actual,KF1,KF2,KF3,MMAEKF");
    Serial.print(actualMotorPosition.get());
    Serial.print(",");
    Serial.print(KF1MotorPosition.get());
    Serial.print(",");
    Serial.print(KF2MotorPosition.get());
    Serial.print(",");
    Serial.print(KF3MotorPosition.get());
    Serial.print(",");
    Serial.println(MMAEMotorPosition.get());

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
    int16_t sliderValue = analogRead(SLIDER_PIN);
    sliderValue = sliderValue * 100 / 3500 + 10;
    sliderPosition.put(sliderValue);
    // float mspeed = sliderValue;
    // motorSpeed.put(mspeed * 4);
    // Serial.print("Slider Value:");
    // Serial.println(sliderValue);
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
    // accelerometerReading.put(getIMU_y);
    // Serial.print(",IMU x: ");
    // Serial.print(getIMU_x());
    Serial.print(",IMU y:");
    Serial.print(getIMU_y());
    // Serial.println(",IMU z:");
    // Serial.print(getIMU_z());
    vTaskDelay(100); // Task period
  }
}
//********************************************************************************
// Limit switch task
void limitSwitchTask(void *p_params)
{
  while (true)
  {
    bool limitSwitchStatus = digitalRead(LIM_PIN);
    if (limitSwitchStatus == LOW) // TODO: check if this is correct
    {
      limitSwitchPressed.put(true);
      encoder.setCount(0);
    }
    else
    {
      limitSwitchPressed.put(false);
    }
    vTaskDelay(100); // Task period
  }
}
//********************************************************************************
// Acoustic positioning task
void acousticTask(void *p_params)
{
  float spacing = 65;     // spacing between receivers, mm
  float zero_dist = 381;  // spacing between transmitter and receiver in straight line, mm
  float x_r = 0.0;        // calculated position of platform from right sensor
  float x_l = 0.0;        // calculated position of platform from left sensor
  float x = 0.0;          // average calculated position
  float d_r = 0.0;        // calculated distance from right sensor to transmitter
  float d_l = 0.0;        // calculated distance from left sensor to transmitter
  float c = 145;          // calibration constant for distance offset of ultrasonic sensors
  float m = 0.684;        // calibration constant for slope correction of ultrasonic sensors
  unsigned long startTime = 0;
  attachInterrupt(REC_R, rightInt, FALLING);
  attachInterrupt(REC_L, leftInt, FALLING);
  while (true)
  {
    digitalWrite(TRANS, HIGH);  // turn transmitter on for 10 microseconds, then turn off
    delayMicroseconds(10);
    digitalWrite(TRANS, LOW);
    startTime = micros();       // start timer when pulse is sent
    vTaskDelay(100);            // wait 100ms for pulse to be delivered
    
    if (leftTime.get != 0)      // if left sensor gives reading
    {
      d_r = float((rightTime.get() - startTime) * 0.343 * m + c); // distance is difference in time between start time and sensor...
      d_l = float((leftTime.get() - startTime) * 0.343 * m + c);  // measurement time, with corrections from calibration constants
      x_r = sqrt(abs(d_r*d_r - zero_dist*zero_dist)) + spacing/2; // assume vertical distance from sensors to transmitter is known...
      x_l = sqrt(abs(d_l*d_l - zero_dist*zero_dist)) - spacing/2; // and construct two right triangles to get x position of platform...
      x = (x_r + x_l)/2;                                          // centered between receivers, then take average
      acousticPosition.put(x);
      Serial.print("Acoustic position (mm): ");
      Serial.println(x);
    }
    else
    {
      Serial.println("Error! Acoustic not working");
    }
    x_r = 0.0;
    x_l = 0.0;
    x = 0.0;
    d_r = 0.0;
    d_l = 0.0;
    rightTime.put(0);
    leftTime.put(0);
    vTaskDelay(100);
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
  pinMode(ENA_PIN, INPUT);
  pinMode(ENB_PIN, INPUT);
  pinMode(LIM_PIN, INPUT_PULLUP);
  pinMode(SLIDER_PIN, INPUT);
  // ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachFullQuad(ENA_PIN, ENB_PIN);
  encoder.setCount(0);

  // Initialize shared variables
  motorSpeed.put(0);
  limitSwitchPressed.put(false);

  // Setup IMU
  setupIMU();

  // Initialize OLED
  // Wire.begin(OLED_SDA, OLED_SCL);
  // if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  // {
  //   Serial.println(F("OLED not found"));
  //   while (1)
  //     ;
  // }

  /* Start FreeRTOS tasks */
  // xTaskCreate(displayTask, "Display Task", 10000, NULL, 2, NULL);
  xTaskCreate(controlInputTask, "Control input Task", 4096, NULL, 2, NULL);
  // xTaskCreate(accelerometerTask, "Accelerometer Task", 4096, NULL, 3, NULL);
  xTaskCreate(limitSwitchTask, "Limit Switch Task", 4096, NULL, 2, NULL);
  xTaskCreate(motorTask, "Motor Task", 8192, NULL, 3, NULL);
  xTaskCreate(encoderTask, "Encoder Task", 4096, NULL, 4, NULL);
  xTaskCreate(ssControllerTask, "State-Space Controller Task", 8192, NULL, 5, NULL);
  // xTaskCreate(arduinoTask, "Arduino Communication Task", 4096, NULL, 3, NULL);
  // xTaskCreate(accelerometerTask, "IMU", 4096, NULL, 3, NULL);
  // xTaskCreate(KF1Task, "KF1 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(KF2Task, "KF2 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(KF3Task, "KF3 Task", 10000, NULL, 3, NULL);
  // xTaskCreate(MMAETask, "MMAE Task", 10000, NULL, 3, NULL);
  xTaskCreate(acousticTask, "Acoustic Task", 4096, NULL, 2, NULL);
}

/**
 * @brief Main loop that does nothing (ensures FreeRTOS does not crash)
 *
 */
void loop()
{
  delay(60000);
};