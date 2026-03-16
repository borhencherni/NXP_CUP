#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#include "Config.h"
#include "LineDetector.h"


Pixy2 pixy;
Servo servo;


float filteredSteering  = 0.0f;
float integralError     = 0.0f;     //  PID integral term
float lastError         = 0.0f;     //  PID derivative term
float lastSteeringAngle = SERVO_CENTER;


void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  pixy.init();
  pixy.changeProg("line");
  pixy.setLamp(1, 1);
  servo.attach(19);
  servo.write(SERVO_CENTER);

}
/*void sendDataToESP(float vx, float vy, float steeringangle, float servoangle)
{
  Serial3.print("D,");   
  Serial3.print(vx,4);
  Serial3.print(",");

  Serial3.print(vy,4);
  Serial3.print(",");

  Serial3.print(steeringangle,2);
  Serial3.print(",");

  Serial3.print(servoangle,2);

  Serial3.println();   
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

float vectorWeight(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrt(dx*dx + dy*dy);

  float proximity = pixy.line.vectors[i].m_y0 / (float)pixy.frameHeight;

  return length * proximity;
}
bool validVector(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrt(dx*dx + dy*dy);

  if(length < MIN_VECTOR_LEN)
    return false;

  float angle = atan2(dy,dx) * 180 / PI;

  if(abs(angle) < MIN_VECTOR_ANGLE)
    return false;

  return true;
}
void fusedVector(float &vx, float &vy)
{
  vx = 0.0f;
  vy = 0.0f;

  for(int i=0;i<pixy.line.numVectors;i++)
  {
    if(!validVector(i))
    continue;
    float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
    float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

    float w = vectorWeight(i);

    vx += dx * w;
    vy += dy * w;
  }
}
void normalizeFusedVector(float &vx, float &vy)
{
  float mag = sqrt(vx*vx + vy*vy);

  if(mag > 1e-6f)
  {
    vx /= mag;
    vy /= mag;
  }
}
float adaptiveLookahead(float vx) {
    // |vx| ∈ [0,1] after normalization: 0=straight, 1=sharp turn
    float curvature = fabsf(vx);
    // Linearly interpolate: more curvature → shorter lookahead
    return L_MAX - curvature * (L_MAX - L_MIN);
}

void lookaheadPoint(float vx, float vy, float& px, float& py) {
    float L = adaptiveLookahead(vx);   // UPDATE 2: was fixed 0.6
    px = pixy.frameWidth  / 2.0f + vx * L * pixy.frameWidth;
    py = pixy.frameHeight        + vy * L * pixy.frameHeight;
}

float computeSteering(float px, float dt) {
    float error = px - pixy.frameWidth / 2.0f;

    float p = KP * error;

    integralError += error * dt;
    integralError  = constrain(integralError, -I_MAX, I_MAX);
    float i   = KI * integralError;
    
    float d = KD * (error - lastError) / dt;
    lastError   = error;

    // Convert total PID output (pixels) → degrees via atan2 (same as original)
    float output = p + i + d;
    return atan2f(output, 40.0f) * 180.0f / PI;
}*/
static unsigned long lastTime = 0;

void loop() {
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;  
  lastTime = now;

  pixy.line.getAllFeatures();

  if (pixy.line.numVectors == 0)
  {
    servo.write(90);
    Serial.println("No line detected");
    return;
  }
  float vx, vy;
  LineDetector detLine = LineDetector(pixy);
  if(detLine.update()) {
    detLine.getFusedVector(vx, vy);
  } else {
    vx = 0.0f;
    vy = 0.0f;
  };
  
  
  
  /*normalizeFusedVector(vx, vy);
  /*float px, py;
  lookaheadPoint(vx, vy, px, py);
  float steering = computeSteering(px, dt); 
  steering = constrain(steering, -20.0f, 20.0f); 
  filteredSteering = LPF_ALPHA * filteredSteering + (1.0f - LPF_ALPHA) * steering;
  float servoAngle = 90 + filteredSteering;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  sendDataToESP(vx, vy, filteredSteering, servoAngle);
  servo.write(servoAngle);
  Serial.print("Steering angle: ");
  Serial.print(filteredSteering);
  Serial.print("  Servo angle: ");
  Serial.println(servoAngle);
  /*pixy.line.getAllFeatures();

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

  float steering = computeSteering(px, 0.1); // Assuming a time step of 0.1 seconds

  steering = constrain(steering, -35, 35);

  filteredSteering = 0.7 * filteredSteering + 0.3 * steering;

  servo.write(90 + filteredSteering);*/
}
