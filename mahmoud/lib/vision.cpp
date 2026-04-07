#include "vision.h"
#include <Arduino.h>
#include <math.h>

Pixy2 pixy;

void Vision_Init()
{
    pixy.init();
    pixy.changeProg("line");

    Serial.println("Pixy Line Tracking Initialized");
    pixy.setLamp(1, 1);
}

float Vision_GetAngleError()
{
    int8_t res;

    res = pixy.line.getAllFeatures();

    if (res <= 0)
        return 0;

    if (pixy.line.numVectors > 0)
    {
        int x0 = pixy.line.vectors[0].m_x0;
        int y0 = pixy.line.vectors[0].m_y0;
        int x1 = pixy.line.vectors[0].m_x1;
        int y1 = pixy.line.vectors[0].m_y1;

        float dx = (float)(x1 - x0);
        float dy = (float)(y1 - y0);

        // angle relative to vertical axis
        float angle = atan2(dx, dy);

        float angle_deg = angle * 180.0 / PI;

        return angle_deg;
    }

    return 0;
}

float Vision_GetAngleError()
{
    int8_t res;

    res = pixy.line.getAllFeatures();

    if (res <= 0)
        return 0;

    if (pixy.line.numVectors > 0)
    {
        int x0 = pixy.line.vectors[0].m_x0;
        int y0 = pixy.line.vectors[0].m_y0;

        int x1 = pixy.line.vectors[0].m_x1;
        int y1 = pixy.line.vectors[0].m_y1;

        float angle = atan2((float)(y1 - y0), (float)(x1 - x0));

        float angle_deg = angle * 180.0 / PI;

        return angle_deg;
    }

    return 0;
}

void Vision_PrintFeatures()
{
    int8_t res;

    res = pixy.line.getAllFeatures();

    if (res <= 0)
        return;

    Serial.println("Vectors:");

    for (int i = 0; i < pixy.line.numVectors; i++)
    {
        Serial.print("x0: ");
        Serial.print(pixy.line.vectors[i].m_x0);

        Serial.print(" x1: ");
        Serial.print(pixy.line.vectors[i].m_x1);

        Serial.print(" y0: ");
        Serial.print(pixy.line.vectors[i].m_y0);

        Serial.print(" y1: ");
        Serial.println(pixy.line.vectors[i].m_y1);
    }
}