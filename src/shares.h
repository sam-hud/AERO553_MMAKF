/**
 * @file shares.h
 * @author Dylan Ruiz
 * @brief Interface for defining the variables shared between tasks
 * @version 1.0
 * @date 2022-12-06
 *
 */
#ifndef _SHARES_H_
#define _SHARES_H_

#include "taskqueue.h"
#include "taskshare.h"

extern Share<bool> stopMotor1;
extern Share<bool> stopMotor2;
extern Share<bool> beginMove;
extern Queue<float> directionsQueue;
extern Share<uint16_t> steps1;
extern Share<uint16_t> steps2;
extern Share<float> velocity1;
extern Share<float> velocity2;
extern Share<uint8_t> startMotor1;
extern Share<uint8_t> startMotor2;
extern Share<int8_t> dirMotor1;
extern Share<int8_t> dirMotor2;
extern Share<bool> moveComplete;
extern Share<bool> startLimitx;
extern Share<bool> startLimity;
#endif // _SHARES_H_
