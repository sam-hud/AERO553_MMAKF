/**
 * @file Controller.h
 * @author Sam Hudson
 * @brief Provides the Main FSM, controls the actuator,
 * reads the IR sensor, and coordinates the movement of both motors
 * @version 1.0
 * @date 2022-11-10
 */

#ifndef _CONTROLLER_TASK_H_
#define _CONTROLLER_TASK_H

#include <Arduino.h>
#include "taskqueue.h"
#include "taskshare.h"
#include "objects/MotorDriver.h"

class Controller
{
private:
    uint8_t state;
    uint8_t SOLENOID_PIN;
    uint8_t SENSOR_PIN;
    float stepLength; // deg/step
    float xCoordinateFrom;
    float yCoordinateFrom;
    float xCoordinateTo;
    float yCoordinateTo;
    float takePiece;
    float xPieceGraveyard;
    float yPieceGraveyard;
    float sensorOffset;
    uint8_t count = 0;
    float moveTake;

    bool stateFlag2;
    bool stateFlag3;
    bool stateFlag6;
    bool stateFlag8;
    bool stateFlag10;
    bool stateFlag11;
    bool stateFlag12;
    bool stateFlag13;
    void origin_x();
    void origin_y();
    void movePiece(float moveFromx, float moveFromy, float moveTox, float moveToy);
    void grabPiece();
    void centerToGrid();
    void gridToGraveyard();
    void xGridMove(uint16_t x_to, uint16_t x_from);
    void yGridMove(uint16_t y_to, uint16_t y_from);
    void gridToCenter();
    void releasePiece();
    bool detectPiece();

public:
    Controller(uint8_t SOLENOID_PIN, uint8_t SENSOR_PIN); // Constructor
    void run();                                                                  // Method for FSM
};
#endif // _CONTROLLER_TASK_H_
