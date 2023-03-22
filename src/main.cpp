#include <Arduino.h>
#include <Wire.h>                 // This library allows you to communicate with I2C devices.
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_SSD1306.h>     // Hardware-specific library for SSD1306 displays
#include <HardwareSerial.h>       // Serial communication
#include "Motor.h"                // Header for motor class
#include "QueueShare/taskshare.h" // Header for inter-task shared data
#include "QueueShare/taskqueue.h" // Header for inter-task data queues
#include <ESP32Encoder.h>
#include "IMU.h"
#include <StateSpaceControl.h>
#include <SimpleKalmanFilter.h> // Kalman Filter library

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
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

/* Define Shares*/
Share<float> motorSpeed("Motor Speed");
Share<bool> limitSwitchPressed("Limit Switch Pressed");
Share<int16_t> sliderPosition("Slider Position");
Share<float> actualMotorPosition("Actual Motor Position");
Share<float> KF1PositionEstimate("KF1 Position");
Share<float> KF2PositionEstimate("KF2 Position");
Share<float> KF3PositionEstimate("KF3 Position");
Share<float> acousticPositionEstimate("Acoustic Position");
Share<float> MMAEPositionEstimate("MMAE Position");
Share<float> accelerometerReading("Accelerometer Reading");
Share<int16_t> encPos("Encoder position");
Share<unsigned long> rightTime("Right Acoustic Time");
Share<unsigned long> leftTime("Left Acoustic Time");
Share<float> acousticPosition("Acoustic Position");
Queue<float> acousticTunerMeasured(50, "Acoustic Tuner Measured");
Queue<float> acousticTunerCalculatedR(50, "Acoustic Tuner Calculated R");
Queue<float> acousticTunerCalculatedL(50, "Acoustic Tuner Calculated L");
Share<float> accelReadings("Accelerometer Readings");
Queue<float> velReadings(10, "Velocity Readings");
Share<float> accelPosReading("Accelerometer Position Reading");
// Share<String> MMAEProp("MMAE Proportions");
Queue<float> IMUTunerCalculated(50, "IMU Tuner Calculated");
Queue<float> IMUTunerMeasured(50, "IMU Tuner Measured");

// Create each motor driver object
Motor motor(PWM_PIN, DIR_PIN, BRK_PIN);

// Create encoder
ESP32Encoder encoder;

// Create Kalman Filters. Change constants here
// (Measurement Uncertainty, Estimation Uncertainty, Process Noise)
SimpleKalmanFilter KF1(2, 2, 0.01);
SimpleKalmanFilter KF2(2, 2, 0.01);
SimpleKalmanFilter KF3(2, 2, 0.01);
SimpleKalmanFilter ACOUSTICKF(2, 2, 0.01);
SimpleKalmanFilter IMU(0.001, 0.0001, 0.001);

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
  accelPosReading.put(0);
  accelReadings.put(0);
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
    // Serial.println(gain);
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

    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(WHITE);

    // display.setCursor(0, 0);
    // display.println("MMAKF");
    // display.setTextSize(1);

    // display.display();

    /* Serial monitor output */
    Serial.println("\n \n \n");
    Serial.println("-----------------------------");
    Serial.print("        Desired position (mm): ");
    Serial.println(sliderPosition.get());
    Serial.print("         Actual position (mm): ");
    Serial.println(actualMotorPosition.get());
    Serial.print("       Acoustic position (mm): ");
    Serial.println(acousticPosition.get());
    Serial.print("Kalman acoustic position (mm): ");
    Serial.println(acousticPositionEstimate.get());
    Serial.print("         Accel. position (mm): ");
    Serial.println(accelPosReading.get());
    Serial.print(" Accel. KF1 position est (mm): ");
    Serial.println(KF1PositionEstimate.get());
    Serial.print(" Accel. KF2 position est (mm): ");
    Serial.println(KF2PositionEstimate.get());
    Serial.print(" Accel. KF3 position est (mm): ");
    Serial.println(KF3PositionEstimate.get());
    Serial.print("       MMAE position est (mm): ");
    Serial.println(MMAEPositionEstimate.get());
    // Serial.print("              MMAE proportion: ");
    // Serial.println(MMAEProp.get());
    Serial.println("-----------------------------");

    vTaskDelay(500); // Task period
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
    if (sliderValue > 2200)
      sliderValue = 2200;
    sliderValue = sliderValue * 200 / 3500 + 20;
    sliderPosition.put(sliderValue);
    // float mspeed = sliderValue;
    // motorSpeed.put(mspeed * 4);
    // Serial.print("Slider Value:");
    // Serial.println(sliderValue);
    vTaskDelay(1); // Task period
  }
}
//********************************************************************************
// Accelerometer reading task
void accelerometerTask(void *p_params)
{
  const TickType_t xFrequency = 1;
  TickType_t xLastWakeTime;
  while (true)
  {
    xLastWakeTime = xTaskGetTickCount();
    float a = (-1 * getIMU_y() * 9.81 / 16384 + 0.41); // m/s^2
    accelReadings.put(a);
    // Serial.print("Accel:");
    // Serial.println(a);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
void accelToVelTask(void *p_params)
{
  const TickType_t xFrequency = 1;
  TickType_t xLastWakeTime;
  float last = 0;
  while (true)
  {
    xLastWakeTime = xTaskGetTickCount();
    float v = 0;
    float now = 0;
    int8_t dt = 1;
    for (int i = 0; i < 10; i++)
    {
      now = accelReadings.get();
      v += (now + last) * 0.5 * dt;
      // Serial.println(a);
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      last = now;
    }
    velReadings.put(v);

    // Serial.print("Velo:");
    // Serial.println(v);
  }
}

void velToPosTask(void *p_params)
{
  int8_t dt = 10;
  const TickType_t xFrequency = 1;
  TickType_t xLastWakeTime;
  float last = 0;
  float m = 0.20892;
  float b = 4.9602;
  while (true)
  {
    xLastWakeTime = xTaskGetTickCount();
    float now = 0;
    float x = 0;
    for (int i = 0; i < 10; i++)
    {
      now = velReadings.get();
      x += (now + last) * 0.5 * dt;
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      last = now;
    }
    x = m*x + b;
    accelPosReading.put(x);
    // IMUTunerCalculated.put(x);
    // IMUTunerMeasured.put(actualMotorPosition.get());
    // Serial.print("Pos:");
    // Serial.println(x);
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
  float spacing = 65;    // spacing between receivers, mm
  float zero_dist = 381; // spacing between transmitter and receiver in straight line, mm
  float x_r = 0.0;       // calculated position of platform from right sensor
  float x_l = 0.0;       // calculated position of platform from left sensor
  float x = 0.0;         // average calculated position
  float d_r = 0.0;       // calculated distance from right sensor to transmitter
  float d_l = 0.0;       // calculated distance from left sensor to transmitter
  float c_r = -264.48;   // calibration constant for distance offset of ultrasonic sensors
  float m_r = 0.57754;   // calibration constant for slope correction of ultrasonic sensors
  float c_l = -361.91;   // calibration constant for distance offset of ultrasonic sensors
  float m_l = 0.66979;   // calibration constant for slope correction of ultrasonic sensors
  float p = 0.44;
  int8_t sign = 0;
  // float d_r_sum = 0;
  // float d_l_sum = 0;
  // uint8_t n = 0;
  unsigned long startTime = 0;
  while (true)
  {
    digitalWrite(TRANS, HIGH); // turn transmitter on for 10 microseconds, then turn off
    delayMicroseconds(10);
    digitalWrite(TRANS, LOW);
    startTime = micros(); // start timer when pulse is sent
    vTaskDelay(100);      // wait 100ms for pulse to be delivered

    if (leftTime.get() != 0) // if left sensor gives reading
    {
      d_r = float((rightTime.get() - startTime) * 0.343 * m_r + c_r); // distance is difference in time between start time and sensor...
      d_l = float((leftTime.get() - startTime) * 0.343 * m_l + c_l);  // measurement time, with corrections from calibration constants
      sign = (actualMotorPosition.get() - spacing / 2) / abs(actualMotorPosition.get() - spacing / 2);
      x_r = sign * sqrt(abs(d_r * d_r - zero_dist * zero_dist)) + spacing / 2; // assume vertical distance from sensors to transmitter is known...
      x_l = sqrt(abs(d_l * d_l - zero_dist * zero_dist)) - spacing / 2;        // and construct two right triangles to get x position of platform...
      x = (x_r * p + x_l * (1 - p));                                           // centered between receivers, then take average
      acousticPosition.put(x);
      // acousticTunerCalculatedL.put(d_l);
      // acousticTunerCalculatedR.put(d_r);
      // acousticTunerMeasured.put(actualMotorPosition.get());
      // Serial.println("-----------------------------");
      // Serial.print("        Desired position (mm): ");
      // Serial.println(sliderPosition.get());
      // Serial.print("         Actual position (mm): ");
      // Serial.println(actualMotorPosition.get());
      // Serial.print("       Acoustic position (mm): ");
      // Serial.println(acousticPosition.get());
      // Serial.print("            IMU position (mm): ");
      // Serial.println(accelPosReading.get());
      // Serial.print("Kalman acoustic position (mm): ");
      // Serial.println(acousticPositionEstimate.get());
      // Serial.println("---------------");
      // Serial.print(x_l + spacing / 2);
      // Serial.print(", ");
      // Serial.print(x_r - spacing / 2);
      // Serial.print(", ");
      // Serial.println(sign);
      // Serial.println("---------------");
    }
    else
    {
      Serial.println("Error! Acoustic not working");
    }
    // if (n < 50)
    // {
    //   d_r_sum += d_r;
    //   d_l_sum += d_l;
    //   n++;
    // }
    // else
    // {
    //   Serial.println(d_l);
    //   Serial.println(d_r);
    //   Serial.println("REAL BELOW");
    //   d_r = d_r_sum / n;
    //   d_l = d_l_sum / n;
    //   Serial.println(d_l);
    //   Serial.println(d_r);
    //   Serial.println("");
    //   d_r_sum = 0;
    //   d_l_sum = 0;
    //   n = 0;
    // }
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

void acousticTunerTask(void *p_params)
{
  float new_meas = 0.0;
  float new_xl = 0.0;
  float new_xr = 0.0;
  float new_yl = 0.0;
  float new_yr = 0.0;
  uint16_t n = 0;
  float sum_xl = 0.0;
  float sum_x2l = 0.0;
  float sum_yl = 0.0;
  float sum_xyl = 0.0;
  float sum_xr = 0.0;
  float sum_x2r = 0.0;
  float sum_yr = 0.0;
  float sum_xyr = 0.0;
  float ml = 0.0;
  float bl = 0.0;
  float mr = 0.0;
  float br = 0.0;
  while (true)
  {
    if (n >= 100)
    {
      // Serial.print(sum_xyl);
      // Serial.print(", ");
      // Serial.print(sum_xl);
      // Serial.print(", ");
      // Serial.print(sum_yl);
      // Serial.print(", ");
      // Serial.print(sum_x2l);
      // Serial.print(", ");
      // Serial.print(sum_xyr);
      // Serial.print(", ");
      // Serial.print(sum_xr);
      // Serial.print(", ");
      // Serial.print(sum_yr);
      // Serial.print(", ");
      // Serial.println(sum_x2r);
      ml = (n * sum_xyl - sum_xl * sum_yl) / (n * sum_x2l - sum_xl * sum_xl);
      mr = (n * sum_xyr - sum_xr * sum_yr) / (n * sum_x2r - sum_xr * sum_xr);
      bl = (sum_yl - ml * sum_xl) / n;
      br = (sum_yr - mr * sum_xr) / n;
      Serial.print("m_l*1000: ");
      Serial.print(ml * 1000);
      Serial.print(", m_r*1000: ");
      Serial.print(mr * 1000);
      Serial.print(", b_l*1000: ");
      Serial.print(bl * 1000);
      Serial.print(", b_r*1000: ");
      Serial.println(br * 1000);
      n = 0;
      sum_xl = 0.0;
      sum_x2l = 0.0;
      sum_yl = 0.0;
      sum_xyl = 0.0;
      sum_xr = 0.0;
      sum_x2r = 0.0;
      sum_yr = 0.0;
      sum_xyr = 0.0;
      ml = 0.0;
      bl = 0.0;
      mr = 0.0;
      br = 0.0;
    }
    if (acousticTunerCalculatedL.any())
    {
      acousticTunerCalculatedL.get(new_xl);
      acousticTunerCalculatedR.get(new_xr);
      acousticTunerMeasured.get(new_meas);
      new_yl = sqrt(381 * 381 + (new_meas + 65 / 2) * (new_meas + 65 / 2));
      new_yr = sqrt(381 * 381 + (new_meas - 65 / 2) * (new_meas - 65 / 2));
      sum_xl = sum_xl + new_xl;
      sum_xr = sum_xr + new_xr;
      sum_yl = sum_yl + new_yl;
      sum_yr = sum_yr + new_yr;
      sum_x2l = sum_x2l + new_xl * new_xl;
      sum_x2r = sum_x2r + new_xr * new_xr;
      sum_xyl = sum_xyl + new_xl * new_yl;
      sum_xyr = sum_xyr + new_xr * new_yr;
      n++;
      // Serial.print("new_yl: ");
      // Serial.print(new_yl);
      // Serial.print(", new_yr: ");
      // Serial.print(new_yr);
      // Serial.print(", new_xl: ");
      // Serial.print(new_xl);
      // Serial.print(", new_xr: ");
      // Serial.println(new_xr);
    }
    vTaskDelay(100);
  }
}

void IMUTunerTask(void *p_params)
{
  float new_x = 0.0;
  float new_y = 0.0;
  float last_y = 0.0;
  uint16_t n = 0;
  float sum_x = 0.0;
  float sum_x2 = 0.0;
  float sum_y = 0.0;
  float sum_xy = 0.0;
  float m = 0.0;
  float b = 0.0;
  float pos = 0.0;
  while (true)
  {
    // IMUTunerCalculated.put(accelPosReading.get());
    // IMUTunerMeasured.put(actualMotorPosition.get());
    pos = (n/20)*20 + 20;
    sliderPosition.put(pos);

    if (n >= 140)
    {
      m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
      b = (sum_y - m * sum_x) / n;
      Serial.println("---------------");
      Serial.print("m*1000: ");
      Serial.print(m * 1000);
      Serial.print(", b*1000: ");
      Serial.println(b * 1000);
      Serial.println("---------------");
      n = 0;
      sum_x = 0.0;
      sum_x2 = 0.0;
      sum_y = 0.0;
      sum_xy = 0.0;
      last_y = 0.0;
      m = 0.0;
      b = 0.0;
    }
    if (IMUTunerCalculated.any())
    {
      last_y = new_y;
      IMUTunerCalculated.get(new_x);
      IMUTunerMeasured.get(new_y);

      if ((new_y < (last_y + 2)) || (new_y > (last_y - 2)))
      {
        if ((new_x > 0) && (new_x < 650))
        {
          sum_x = sum_x + new_x;
          sum_y = sum_y + new_y;
          sum_x2 = sum_x2 + new_x * new_x;
          sum_xy = sum_xy + new_x * new_y;
          n++;
          Serial.print("n: ");
          Serial.print(n);
          Serial.print(", new_y: ");
          Serial.print(new_y );
          Serial.print(", new_x: ");
          Serial.println(new_x);
        }
      }
    }
    vTaskDelay(50);
  }
}

//********************************************************************************
// Kalman filter task
void KalmanFilterTask(void *p_params)
{
  while (true)
  {
    // Compares each KF to acoustic measurement and weights them based on their error

    // Get accelerometer and acoustic reading
    float accelReading = accelPosReading.get();
    float acousticReading = acousticPosition.get();

    // Update KFs and store estimates
    float KF1Estimate = KF1.updateEstimate(accelReading);
    KF1PositionEstimate.put(KF1Estimate);
    float KF2Estimate = KF2.updateEstimate(accelReading);
    KF2PositionEstimate.put(KF2Estimate);
    float KF3Estimate = KF3.updateEstimate(accelReading);
    KF3PositionEstimate.put(KF3Estimate);

    float acousticEstimate = ACOUSTICKF.updateEstimate(acousticReading);
    acousticPositionEstimate.put(acousticEstimate);

    // Calculate errors
    float KF1Error = abs(KF1Estimate - acousticEstimate);
    float KF2Error = abs(KF2Estimate - acousticEstimate);
    float KF3Error = abs(KF3Estimate - acousticEstimate);

    // Calculate total error
    float totalError = KF1Error + KF2Error + KF3Error;

    // Calculate weights
    float KF1Weight = 1 - KF1Error / totalError;
    float KF2Weight = 1 - KF2Error / totalError;
    float KF3Weight = 1 - KF3Error / totalError;

    // Calculate MMAE output
    float MMAEOutput = (KF1Estimate * KF1Weight) + (KF2Estimate * KF2Weight) + (KF3Estimate * KF3Weight);
    MMAEPositionEstimate.put(MMAEOutput);
    // String prop = String(KF1Weight) + "," + String(KF2Weight) + "," + String(KF3Weight);
    // MMAEProp.put(prop);

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
  pinMode(TRANS, OUTPUT);
  pinMode(REC_L, INPUT);
  pinMode(REC_R, INPUT);
  // ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachFullQuad(ENA_PIN, ENB_PIN);
  encoder.setCount(0);
  attachInterrupt(REC_R, rightInt, FALLING);
  attachInterrupt(REC_L, leftInt, FALLING);

  // Initialize shared variables
  motorSpeed.put(0);
  leftTime.put(0);
  rightTime.put(0);
  limitSwitchPressed.put(false);
  acousticPositionEstimate.put(0);
  accelerometerReading.put(0);
  acousticPosition.put(0);
  accelReadings.put(0);
  velReadings.put(0);
  accelPosReading.put(0);
  // MMAEProp.put("0,0,0");

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
  /*------ Platform moving tasks ------*/
  xTaskCreate(limitSwitchTask, "Limit Switch Task", 4096, NULL, 2, NULL);
  xTaskCreate(motorTask, "Motor Task", 8192, NULL, 3, NULL);
  xTaskCreate(encoderTask, "Encoder Task", 4096, NULL, 4, NULL);
  xTaskCreate(ssControllerTask, "State-Space Controller Task", 8192, NULL, 5, NULL);
  xTaskCreate(controlInputTask, "Control input Task", 4096, NULL, 2, NULL);

  /*------ MMAE KF tasks ------*/
  xTaskCreate(displayTask, "Display Task", 10000, NULL, 2, NULL);
  xTaskCreate(accelerometerTask, "Accelerometer Task", 10000, NULL, 3, NULL);
  xTaskCreate(acousticTask, "Acoustic Task", 4096, NULL, 2, NULL);
  // xTaskCreate(acousticTunerTask, "Acoustic Tuner Task", 4096, NULL, 3, NULL);
  xTaskCreate(KalmanFilterTask, "MMAE Task", 10000, NULL, 3, NULL);
  xTaskCreate(accelToVelTask, "Accel to Vel Task", 4096, NULL, 2, NULL);
  xTaskCreate(velToPosTask, "Vel to Pos Task", 4096, NULL, 2, NULL);
  // xTaskCreate(IMUTunerTask, "IMU Tuner Task", 4096, NULL, 3, NULL);
}

/* Main loop that does nothing (ensures FreeRTOS does not crash) */
void loop()
{
  delay(60000);
};