#include <Arduino.h>

#include "esp_telemetry.h"
#include "esp_web.h"

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXTeensy, TXTeensy);
  EspTelemetry_Init();
  EspWeb_Init();
  Serial.println("ESP32 ready, listening for Teensy and serving web UI...");
}

void loop() {
  EspTelemetry_Loop();
  EspWeb_Loop();
}