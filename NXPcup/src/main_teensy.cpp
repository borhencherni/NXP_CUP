#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>

typedef struct vectorPixy
{
  double longueur;
  int m_x1;
  int m_x0;
  int m_y1;
  int m_y0;
} vectorPixy;
Pixy2 pixy;
Servo servo;

void setup() {
  pixy.init();
  pixy.changeProg("line");
  pixy.setLamp(1, 1);
}

void normalizeVectors()
{
  for(int i=0;i<pixy.line.numVectors;i++)
  {
    if(pixy.line.vectors[i].m_y0 < pixy.line.vectors[i].m_y1)
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
void normalizeFusedVector(float &vx, float &vy)
{
  float mag = sqrt(vx*vx + vy*vy);

  if(mag > 0)
  {
    vx /= mag;
    vy /= mag;
  }
}
float vectorWeight(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0;

  float length = sqrt(dx*dx + dy*dy);

  float proximity = pixy.line.vectors[i].m_y0 / (float)pixy.frameHeight;

  return length * proximity;
}
bool validVector(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0;

  float length = sqrt(dx*dx + dy*dy);

  if(length < 15)
    return false;

  float angle = atan2(dy,dx) * 180 / PI;

  if(abs(angle) < 10)
    return false;

  return true;
}
void fusedVector(float &vx, float &vy)
{
  vx = 0;
  vy = 0;

  for(int i=0;i<pixy.line.numVectors;i++)
  {
    if(!validVector(i))
    continue;
    float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
    float dy = pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0;

    float w = vectorWeight(i);

    vx += dx * w;
    vy += dy * w;
  }
}
void lookaheadPoint(float vx,float vy,float &px,float &py)
{
  float L = 0.6;

  px = pixy.frameWidth/2 + vx * L;
  py = pixy.frameHeight + vy * L;
}
float computeSteering(float px)
{
  float error = px - pixy.frameWidth/2;

  float steering = atan2(error , 40);

  return steering * 180 / PI;
}
float filteredSteering = 0;

void loop() {
  pixy.line.getAllFeatures();

  if (pixy.line.numVectors == 0)
  {
    servo.write(90);
    return;
  }

  normalizeVectors();

  float vx, vy;
  fusedVector(vx, vy);

  normalizeFusedVector(vx, vy);

  float px, py;
  lookaheadPoint(vx, vy, px, py);

  float steering = computeSteering(px);

  steering = constrain(steering, -35, 35);

  filteredSteering = 0.7 * filteredSteering + 0.3 * steering;

  servo.write(90 + filteredSteering);
}
