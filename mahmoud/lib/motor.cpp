#include "motor.h"
#include <Servo.h>

// -------- PIN CONFIGURATION --------

// Right motor
#define IN1 21
#define IN2 20

// Left motor
#define IN3 23
#define IN4 22

// Servo
#define SERVO_PIN 19

// Servo object
Servo steeringServo;

void Motor_Init()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    steeringServo.attach(SERVO_PIN);

    Motor_Stop();

    Serial.println("Motors Initialized");
}

void Motor_SetSpeed(int leftSpeed, int rightSpeed)
{
    // LEFT MOTOR
    if (rightSpeed >= 0)
    {
        digitalWrite(IN1, leftSpeed);
        analogWrite(IN2, LOW);
    }
    else
    {
        analogWrite(IN3, -leftSpeed);
        digitalWrite(IN4, LOW);
        
    }

    // RIGHT MOTOR
    if (leftSpeed>= 0)
    {
        analogWrite(IN1, rightSpeed);
        digitalWrite(IN2, HIGH);
    }
    else
    {
        analogWrite(IN3, -rightSpeed);
        digitalWrite(IN4, LOW);
    }
}

void Motor_Stop()
{
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
}

void Motor_SetSteering(int angle)
{
    steeringServo.write(angle);
}