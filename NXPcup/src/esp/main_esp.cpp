#include <Arduino.h>

#define RXTeensy 16   
#define TXTeensy 17   

String buffer = "";

void parsePacket(String packet) {
  // Expected format: D,vx,vy,steeringangle,servoangle
  if (!packet.startsWith("D,")) return;

  float values[4];
  int count = 0;
  String data = packet.substring(2); // strip "D,"

  while (data.length() > 0 && count < 4) {
    int commaIdx = data.indexOf(',');
    String token;

    if (commaIdx == -1) {
      token = data;
      data = "";
    } else {
      token = data.substring(0, commaIdx);
      data  = data.substring(commaIdx + 1);
    }

    values[count++] = token.toFloat();
  }

  if (count < 4) {
    Serial.println("Incomplete packet, skipping.");
    return;
  }

  float vx            = values[0];
  float vy            = values[1];
  float steeringAngle = values[2];
  float servoAngle    = values[3];

  Serial.print("vx: ");    Serial.print(vx, 4);
  Serial.print(" | vy: "); Serial.print(vy, 4);
  Serial.print(" | steering: "); Serial.print(steeringAngle, 2);
  Serial.print(" | servo: ");    Serial.println(servoAngle, 2);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXTeensy, TXTeensy); 
  Serial.println("ESP32 ready, listening for Teensy...");
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      buffer.trim();
      if (buffer.length() > 0) {
        parsePacket(buffer);
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
}