/**
 * @file Controller.cpp
 * @author Sam Hudson
 * @brief Provides the Main FSM, controls the actuator,
 * reads the IR sensor, and coordinates the movement of both motors
 * @version 1.0
 * @date 2022-11-10
 */

#include <Arduino.h>
#include "shares.h"
#include "tasks/ControllerTask.h"
/**
 * @brief Construct a new Controller object
 *
 * @param SOLENOID_PIN Pin for the solenoid
 * @param SENSOR_PIN Pin for the IR sensor
 * @param kinematics Kinematics object for calculations
 */
Controller::Controller(uint8_t SOLENOID_PIN, uint8_t SENSOR_PIN)
{
    this->SOLENOID_PIN = SOLENOID_PIN;
    this->SENSOR_PIN = SENSOR_PIN;
    state = 0; // Start state = 0
    stepLength = .1;
    xCoordinateFrom = 0;           // Define x coordinate of the piece to be moved
    yCoordinateFrom = 0;           // Define y coordinate of the piece to be moved
    xCoordinateTo = 0;             // Define x coordinate of the piece to be moved to
    yCoordinateTo = 0;             // Define y coordinate of the piece to be moved to
    takePiece = 0;                 // If takePiece = 1, then a piece needs to be taken
    moveTake = 0;                  // If moveTake = 1, then a move to the piece is needed (for FSM control)
    xPieceGraveyard = 520;         // x coordinate of the piece graveyard
    yPieceGraveyard = 522.5 - 180; // y coordinate of the piece graveyard
    sensorOffset = 18.25;          // Offset for the IR sensor to the actuator magnet

    beginMove.put(false); // Ensure that no moves have started
    uint8_t count = 0;

    stateFlag2 = false;
    stateFlag3 = false;
    stateFlag6 = false;
    stateFlag8 = false;
    stateFlag10 = false;
    stateFlag11 = false;
    stateFlag12 = false;
    stateFlag13 = true;
}

/**
 * @brief Main FSM for the controller
 */
void Controller::run() // Method for FSM
{
    switch (state)
    {
    case 0: // Calibrate x axis
    {
        releasePiece(); // activates solenoid before moving under pieces
        origin_x();
        startLimitx.put(true);
        state = 1;
        stateFlag2 = true;
        break;
    }
    case 1: // Check if motors have stopped
    {
        if ((stopMotor1.get() == true, stopMotor2.get() == true))
        {
            if (stateFlag2)
            {
                state = 2;
                stateFlag2 = false;
            }
            else if (stateFlag3)
            {
                state = 3;
                stateFlag3 = false;
            }
            else if (stateFlag6)
            {
                state = 6;
                stateFlag6 = false;
            }
            else if (stateFlag8)
            {
                state = 8;
                stateFlag8 = false;
            }
            else if (stateFlag10)
            {
                state = 10;
                stateFlag10 = false;
            }
            else if (stateFlag11)
            {
                state = 11;
                stateFlag11 = false;
            }
            else if (stateFlag12)
            {
                state = 12;
                stateFlag12 = false;
            }
            else if (stateFlag13)
            {
                state = 13;
                stateFlag13 = false;
            }
        }
        break;
    }
    case 2: // Calibrate y axis
    {
        grabPiece(); // Releases Solenoid Activation
        origin_y();
        startLimity.put(true);
        state = 1; // Wait Motor State
        stateFlag3 = true;
        break;
    }

    case 3: // Check for a move request (waiting state)
    {
        if (moveTake == 1) // If a piece needs finished taking a piece
        {
            state = 5;
            moveTake = 0;
        }
        else if (beginMove.get() == true) // If a move is requested
        {
            state = 4;
            beginMove.put(false);    // Reset the flag
            moveComplete.put(false); // Tell API that move is not complete
        }
        break;
    }
    case 4: // Get new move from FetchMoveTask
    {
        takePiece = directionsQueue.get();       // First val defines if piece needs taking first
        xCoordinateFrom = directionsQueue.get(); // Second val defines x coordinate of piece to move
        yCoordinateFrom = directionsQueue.get(); // Third val defines y coordinate of piece to move
        xCoordinateTo = directionsQueue.get();   // Fourth val defines x coordinate of piece to move to
        yCoordinateTo = directionsQueue.get();   // Fifth val defines y coordinate of piece to move to
        state = 5;
        break;
    }
    case 5: //  Move piece to chess piece
    {
        releasePiece();     // activates solenoid before moving under pieces
        if (takePiece == 1) // If a piece needs taking
        {
            movePiece(0, xCoordinateTo - sensorOffset, 0, yCoordinateTo);
        }
        else
        {
            movePiece(0, xCoordinateFrom - sensorOffset, 0, yCoordinateFrom); // Move to piece
        }
        state = 1;
        stateFlag6 = true;
        break;
    }

    case 6: // Check if sensor is under piece
    {
        if (detectPiece())
        {
            count += 1;
        }
        if (count > 10) // should detect piece for 1 second before initiating move
        {
            state = 7;
        }
        break;
    }

    case 7: // Move actuator under piece
    {
        movePiece(0, sensorOffset, 0, 0); // Move to the right by the sensor offset
        state = 1;
        stateFlag8 = true;
        break;
    }

    case 8: // Grab piece
    {
        grabPiece();
        delay(50);
        state = 9;
        break;
    }

    case 9: // Move to grid before moving along gridlines (along x)
    {
        centerToGrid();
        state = 1;
        stateFlag10 = true;
        break;
    }

    case 10: // Move along x gridline
    {
        if (takePiece == 1) // If piece needs taking
        {
            xGridMove(xPieceGraveyard, xCoordinateTo);
            state = 1;
        }
        else
        {

            xGridMove(xCoordinateTo, xCoordinateFrom);
            state = 1;
        }
        stateFlag11 = true;

        break;
    }

    case 11: // Move along y gridline
    {
        if (takePiece == 1) // If piece needs taking
        {
            yGridMove(yPieceGraveyard, yCoordinateTo);
        }
        else
        {

            yGridMove(yCoordinateTo, yCoordinateFrom);
        }
        state = 1;
        stateFlag12 = true;
        break;
    }

    case 12: // Move piece along x for final position
    {
        if (takePiece) // If piece needs taking
        {
            gridToGraveyard(); // Take piece to graveyard along x
        }
        else
        {
            gridToCenter(); // Move piece to x center of sqaure from grid
        }

        state = 1;
        stateFlag13 = true;
        break;
    }

    case 13: // Release piece
    {
        releasePiece();
        if (takePiece == 0) // Check if piece did not need taking
        {
            moveComplete.put(true); // Tell API that move is complete
        }
        if (takePiece == 1) // If piece needs taking
        {
            moveTake = 1; // Set moveTake to 1 to signal that taken piece was moved
        }
        takePiece = 0; // Reset takePiece flag
        state = 0;
        break;
    }
    }
}

/**
 * @brief Moves the carriage to the x limit switch
 *
 */
void Controller::origin_x()
{
    steps1.put(10000);
    steps2.put(10000);
    dirMotor1.put(1);
    dirMotor2.put(1);
    startMotor1.put(2);
    startMotor2.put(2);
}

/**
 * @brief Moves the carriage to the y limit switch
 *
 */
void Controller::origin_y()
{
    steps1.put(10000);
    steps2.put(10000);
    dirMotor1.put(1);
    dirMotor2.put(-1);
    startMotor1.put(2);
    startMotor2.put(2);
}

/**
 * @brief Moves the carriage to the desired board coordinates
 *
 * @param move
 */
void Controller::movePiece(float move) // State 2
{
    float Dx = moveToX - moveFromX; // mm
    float Dy = moveToY - moveFromY; // mm
    int16_t velocityMotor1 = kinematics.coordsToVelocityMotor1(Dx, Dy);
    int16_t velocityMotor2 = kinematics.coordsToVelocityMotor2(Dx, Dy);
    uint16_t stepsMotor1 = kinematics.coordsToStepsMotor1(Dx, Dy);
    uint16_t stepsMotor2 = kinematics.coordsToStepsMotor2(Dx, Dy);

    steps1.put(stepsMotor1);
    steps2.put(stepsMotor2);
    velocity1.put(velocityMotor1);
    velocity2.put(velocityMotor2);

    startMotor1.put(1);
    startMotor2.put(1);
}