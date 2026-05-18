#include "teensy_vision.h"

#include <Arduino.h>
#include <math.h>

#include "teensy_config.h"
#include "teensy_state.h"

void Vision_Init()
{
  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line");
}

bool isNinetyDegree(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float angle = atan2f(dy, dx) * 180.0f / PI;
  return (fabsf(angle) < 20.0f);
}

float computeMedian(float *arr, int size)
{
  float temp[SERVO_FILTER_SIZE];

  for (int i = 0; i < size; i++)
    temp[i] = arr[i];

  for (int i = 0; i < size - 1; i++)
  {
    for (int j = i + 1; j < size; j++)
    {
      if (temp[j] < temp[i])
      {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[size / 2];
}

void normalizeVectors()
{
  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    if (pixy.line.vectors[i].m_y0 < pixy.line.vectors[i].m_y1)
    {
      int tx = pixy.line.vectors[i].m_x0;
      int ty = pixy.line.vectors[i].m_y0;

      pixy.line.vectors[i].m_x0 = pixy.line.vectors[i].m_x1;
      pixy.line.vectors[i].m_y0 = pixy.line.vectors[i].m_y1;

      pixy.line.vectors[i].m_x1 = tx;
      pixy.line.vectors[i].m_y1 = ty;
    }
  }
}

float vectorWeight(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrtf(dx * dx + dy * dy);
  float proximity = pixy.line.vectors[i].m_y0 / (float)pixy.frameHeight;
  return length * proximity;
}

bool validVector(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrtf(dx * dx + dy * dy);
  if (pixy.line.vectors[i].m_y0 < 15 || pixy.line.vectors[i].m_y1 > pixy.frameHeight - 30)
    return false;
  if (pixy.line.vectors[i].m_x0 < 12 && pixy.line.vectors[i].m_x1 < 12)
    return false;
  if (pixy.line.vectors[i].m_x0 > pixy.frameWidth - 12 && pixy.line.vectors[i].m_x1 > pixy.frameWidth - 12)
    return false;
  if (length < MIN_VECTOR_LEN)
    return false;
  if (length > MAX_VECTOR_LEN)
    return false;

  float angle = atan2f(dy, dx) * 180.0f / PI;
  if (fabsf(angle) < MIN_VECTOR_ANGLE)
    return false;

  return true;
}

void fusedVector(float &vx, float &vy)
{
  vx = 0.0f;
  vy = 0.0f;
  int validCount = 0;
  bool has90 = false;

  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    if (!validVector(i))
      continue;

    if (isNinetyDegree(i))
    {
      has90 = true;
      continue;
    }

    validCount++;

    float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
    float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);
    float w = vectorWeight(i);

    vx += dx * w;
    vy += dy * w;
  }

  if (validCount == 0)
  {
    vx = 0.0f;
    vy = 1.0f;
  }

  if (has90 && validCount > 0)
  {
    vx = 0.0f;
    vy = 1.0f;
  }
}

void normalizeFusedVector(float &vx, float &vy)
{
  float mag = sqrtf(vx * vx + vy * vy);
  if (!isfinite(mag) || mag < 1e-6f)
  {
    vx = 0.0f;
    vy = 1.0f;
    return;
  }

  vx /= mag;
  vy /= mag;
}

float adaptiveLookahead(float vx)
{
  float curvature = fabsf(vx);
  return L_MAX - curvature * (L_MAX - L_MIN);
}

void lookaheadPoint(float vx, float vy, float &px, float &py)
{
  float L = adaptiveLookahead(vx);
  px = pixy.frameWidth / 2.0f + vx * L * pixy.frameWidth;
  py = pixy.frameHeight + vy * L * pixy.frameHeight;
}

float computeSteering(float px, float dt)
{
  float error = px - pixy.frameWidth / 2.0f;

  float p = KP * error;
  integralError += error * dt;
  integralError = constrain(integralError, -I_MAX, I_MAX);
  float i = KI * integralError;

  float d = KD * (error - lastError) / dt;
  lastError = error;

  float output = p + i + d;
  return atan2f(output, 40.0f) * 180.0f / PI;
}

bool isSharpTurn(float steering)
{
  return fabsf(steering) > 15.0f;
}

bool isIntersectionPixy(int i, int j)
{
  float dx1 = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy1 = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float dx2 = pixy.line.vectors[j].m_x1 - pixy.line.vectors[j].m_x0;
  float dy2 = SCALE_Y(pixy.line.vectors[j].m_y1) - SCALE_Y(pixy.line.vectors[j].m_y0);

  float mag1 = sqrtf(dx1 * dx1 + dy1 * dy1);
  float mag2 = sqrtf(dx2 * dx2 + dy2 * dy2);

  if (mag1 < 1e-5 || mag2 < 1e-5)
    return false;

  float dot = (dx1 * dx2 + dy1 * dy2) / (mag1 * mag2);
  return fabsf(dot) < 0.25f;
}

void detectLaneSides()
{
  hasLeft = false;
  hasRight = false;

  int midX = pixy.frameWidth / 2;
  int minY = pixy.frameHeight * 0.5f;

  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    if (!validVector(i))
      continue;

    int x0 = pixy.line.vectors[i].m_x0;
    int y0 = pixy.line.vectors[i].m_y0;

    if (y0 < minY)
      continue;

    if (x0 < midX)
      hasLeft = true;
    else
      hasRight = true;
  }
}

bool isCenterDash(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrtf(dx * dx + dy * dy);
  float angle = atan2f(dy, dx) * 180.0f / PI;

  if (fabsf(angle) > DASH_ANGLE_THRESH)
    return false;
  if (length < DASH_MIN_LEN || length > DASH_MAX_LEN)
    return false;

  int cx0 = pixy.line.vectors[i].m_x0;
  int cx1 = pixy.line.vectors[i].m_x1;
  int leftBound = DASH_CENTER_MARGIN;
  int rightBound = pixy.frameWidth - DASH_CENTER_MARGIN;

  if (cx0 < leftBound || cx0 > rightBound)
    return false;
  if (cx1 < leftBound || cx1 > rightBound)
    return false;

  return true;
}

bool detectDashPair()
{
  int candidates[8];
  int count = 0;

  for (int i = 0; i < pixy.line.numVectors && count < 8; i++)
  {
    if (isCenterDash(i))
      candidates[count++] = i;
  }

  if (count < 1)
    return false;

  for (int a = 0; a < count; a++)
  {
    for (int b = a + 1; b < count; b++)
    {
      int yA = pixy.line.vectors[candidates[a]].m_y0;
      int yB = pixy.line.vectors[candidates[b]].m_y0;
      if (abs(yA - yB) <= DASH_Y_PROXIMITY)
        return true;
    }
  }

  return false;
}

bool updateDashDetection()
{
  if (detectDashPair())
  {
    dashConfidence++;
    if (dashConfidence >= DASH_CONFIRM_FRAMES)
    {
      if (!dashDetected)
      {
        dashDetected = true;
        dashTime = millis();
      }
    }
  }
  else
  {
    dashConfidence = max(0, dashConfidence - 1);
    if (dashConfidence == 0)
      dashDetected = false;
  }

  return dashDetected;
}

bool isWideHorizontal(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrtf(dx * dx + dy * dy);

  if (!isNinetyDegree(i))
    return false;
  if (length < 25.0f)
    return false;
  if (pixy.line.vectors[i].m_y0 < pixy.frameHeight * 0.6f)
    return false;

  return true;
}

bool detectFinishLine()
{
  bool horizontalFound = false;

  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    if (isWideHorizontal(i))
    {
      horizontalFound = true;
      break;
    }
  }

  if (horizontalFound && (hasLeft || hasRight) && dashDetected)
    return true;

  return false;
}