#ifndef TEENSY_MOTORS_H
#define TEENSY_MOTORS_H

#include <Arduino.h>

void Motors_Init();
void runMotors(int LPWM, int RPWM);

#endif