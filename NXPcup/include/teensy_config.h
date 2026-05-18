#ifndef TEENSY_CONFIG_H
#define TEENSY_CONFIG_H

#include <Arduino.h>

#define IN1 22
#define IN2 23
#define IN3 6
#define IN4 7
#define ECHO 19
#define TRIG 18
#define SERVO_PIN 8

#define SCALE_Y(y)        ((y) * 1.53f)
#define KP                0.5f
#define KI                0.0f
#define KD                0.05f
#define SERVO_CENTER      90
#define SERVO_MIN         50
#define SERVO_MAX         150
#define I_MAX             10.0f
#define L_MIN             0.15f
#define L_MAX             0.85f
#define MIN_VECTOR_LEN    11.0f
#define MAX_VECTOR_LEN    110.0f
#define MIN_VECTOR_ANGLE  5.0f
#define STEERING_DEADBAND  4.0f
#define MAX_SERVO_STEP    50.0f
#define CONTROL_PERIOD_MS 30
#define MOTOR_SPEED 180
#define SERVO_FILTER_SIZE 5
#define LEFT_GAIN   15.50f
#define RIGHT_GAIN  16.70f
#define DASH_MAX_LEN        18.0f
#define DASH_MIN_LEN         3.0f
#define DASH_CENTER_MARGIN  19
#define DASH_ANGLE_THRESH   30.0f
#define DASH_Y_PROXIMITY    20
#define DASH_CONFIRM_FRAMES  2

#endif