#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void Motor_Init();

void Motor_SetSpeed(int leftSpeed, int rightSpeed);

void Motor_Stop();

void Motor_SetSteering(int angle);

#endif