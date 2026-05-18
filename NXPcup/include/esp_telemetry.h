#ifndef ESP_TELEMETRY_H
#define ESP_TELEMETRY_H

#include <Arduino.h>

struct TelemetrySample
{
  float vx;
  float vy;
  float steeringAngle;
  float servoAngle;
  float x;
  float y;
  unsigned long timestamp;
};

void EspTelemetry_Init();
void EspTelemetry_Loop();
void EspTelemetry_SendCommand(char command);
String EspTelemetry_BuildJson();

#endif