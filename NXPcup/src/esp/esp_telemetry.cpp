#include "esp_telemetry.h"

#include <Arduino.h>

#include "esp_config.h"

static HardwareSerial &teensySerial = Serial2;
static String inputBuffer;
static TelemetrySample latestSample;
static TelemetrySample pointHistory[TELEMETRY_MAX_POINTS];
static size_t pointCount = 0;
static float pathX = 0.0f;
static float pathY = 0.0f;

static void pushPoint(const TelemetrySample &sample)
{
  if (pointCount < TELEMETRY_MAX_POINTS)
  {
    pointHistory[pointCount++] = sample;
    return;
  }

  for (size_t i = 1; i < TELEMETRY_MAX_POINTS; i++)
    pointHistory[i - 1] = pointHistory[i];

  pointHistory[TELEMETRY_MAX_POINTS - 1] = sample;
}

static bool parsePacket(const String &packet, TelemetrySample &sample)
{
  if (!packet.startsWith("D,"))
    return false;

  float values[4];
  int count = 0;
  String data = packet.substring(2);

  while (data.length() > 0 && count < 4)
  {
    int commaIndex = data.indexOf(',');
    String token;

    if (commaIndex == -1)
    {
      token = data;
      data = "";
    }
    else
    {
      token = data.substring(0, commaIndex);
      data = data.substring(commaIndex + 1);
    }

    values[count++] = token.toFloat();
  }

  if (count < 4)
    return false;

  sample.vx = values[0];
  sample.vy = values[1];
  sample.steeringAngle = values[2];
  sample.servoAngle = values[3];
  sample.timestamp = millis();

  pathX += sample.vx * TELEMETRY_SCALE;
  pathY += sample.vy * TELEMETRY_SCALE;
  sample.x = pathX;
  sample.y = pathY;
  return true;
}

void EspTelemetry_Init()
{
  inputBuffer = "";
  latestSample = {};
  pointCount = 0;
  pathX = 0.0f;
  pathY = 0.0f;
}

void EspTelemetry_Loop()
{
  while (teensySerial.available())
  {
    char c = teensySerial.read();

    if (c == '\n')
    {
      inputBuffer.trim();
      if (inputBuffer.length() > 0)
      {
        TelemetrySample sample;
        if (parsePacket(inputBuffer, sample))
        {
          latestSample = sample;
          pushPoint(sample);
        }
      }
      inputBuffer = "";
    }
    else
    {
      inputBuffer += c;
    }
  }
}

void EspTelemetry_SendCommand(char command)
{
  teensySerial.print("C,");
  teensySerial.println(command);
}

String EspTelemetry_BuildJson()
{
  String json;
  json.reserve(8192);
  json += "{";
  json += "\"latest\":{";
  json += "\"vx\":" + String(latestSample.vx, 4) + ",";
  json += "\"vy\":" + String(latestSample.vy, 4) + ",";
  json += "\"steering\":" + String(latestSample.steeringAngle, 2) + ",";
  json += "\"servo\":" + String(latestSample.servoAngle, 2) + ",";
  json += "\"x\":" + String(latestSample.x, 2) + ",";
  json += "\"y\":" + String(latestSample.y, 2) + ",";
  json += "\"timestamp\":" + String(latestSample.timestamp);
  json += "},";
  json += "\"points\":[";

  for (size_t i = 0; i < pointCount; i++)
  {
    if (i > 0)
      json += ",";

    json += "{";
    json += "\"x\":" + String(pointHistory[i].x, 2) + ",";
    json += "\"y\":" + String(pointHistory[i].y, 2) + ",";
    json += "\"steering\":" + String(pointHistory[i].steeringAngle, 2) + ",";
    json += "\"servo\":" + String(pointHistory[i].servoAngle, 2);
    json += "}";
  }

  json += "]}";
  return json;
}