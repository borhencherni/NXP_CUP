#include "teensy_sensors.h"

#include <Arduino.h>

#include "teensy_config.h"

void Sensors_Init()
{
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

float readUltrasonic()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  float distance = (duration * 0.0343f) / 2.0f;
  return distance;
}