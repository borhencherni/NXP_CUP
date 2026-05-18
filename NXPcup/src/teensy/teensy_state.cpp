#include "teensy_state.h"

Pixy2 pixy;
Servo servo;

bool hasLeft = false;
bool hasRight = false;
bool ishorizontal = false;
float servoBuffer[SERVO_FILTER_SIZE] = {0.0f};
int servoIndex = 0;
bool bufferFilled = false;

float filteredSteering = 0.0f;
float integralError = 0.0f;
float lastError = 0.0f;
float lastSteeringAngle = SERVO_CENTER;
unsigned long startTime = 0;

int dashConfidence = 0;
bool dashDetected = false;
unsigned long dashTime = 0;
unsigned long lastTime = 0;