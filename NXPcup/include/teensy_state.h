#ifndef TEENSY_STATE_H
#define TEENSY_STATE_H

#include <Pixy2.h>
#include <Servo.h>

#include "teensy_config.h"

extern Pixy2 pixy;
extern Servo servo;

extern bool hasLeft;
extern bool hasRight;
extern bool ishorizontal;
extern float servoBuffer[SERVO_FILTER_SIZE];
extern int servoIndex;
extern bool bufferFilled;

extern float filteredSteering;
extern float integralError;
extern float lastError;
extern float lastSteeringAngle;
extern unsigned long startTime;

extern int dashConfidence;
extern bool dashDetected;
extern unsigned long dashTime;
extern unsigned long lastTime;

#endif