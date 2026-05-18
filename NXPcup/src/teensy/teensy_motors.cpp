#include "teensy_motors.h"

#include "teensy_config.h"
#include "teensy_state.h"

static void driveMotor(int forwardPin, int reversePin, int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0)
  {
    analogWrite(forwardPin, speed);
    analogWrite(reversePin, 0);
  }
  else
  {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -speed);
  }
}

void Motors_Init()
{
  servo.attach(SERVO_PIN);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void runMotors(int LPWM, int RPWM)
{
  driveMotor(IN1, IN2, LPWM);
  driveMotor(IN3, IN4, RPWM);
}