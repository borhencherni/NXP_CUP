#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#define IN1 23
#define IN2 22
#define IN3 21
#define IN4 20
#define TXESP 15
#define RXESP 14
Pixy2 pixy;
Servo servo;
// Aspect ratio correction: Pixy2 pixels are not square
// frameWidth=78, frameHeight=51 → real ratio ≈ 1.53
//frameWidth  = 78 pixels   →  covers the full horizontal FOV
//frameHeight = 51 pixels   →  covers the full vertical FOV
//ratio = frameWidth / frameHeight = 78 / 51 ≈ 1.53
//This means 1 pixel of height = 1.53 pixels of width in real-world distance.
#define SCALE_Y(y)        ((y) * 1.53f)
#define KP                0.80f
#define KI                0.0f    
#define KD                0.0f 
#define SERVO_CENTER      90
#define SERVO_MIN         70       
#define SERVO_MAX         110  
#define I_MAX             15.0f   // Integral windup clamp
// Lookahead 
#define L_MIN             0.4f     // short lookahead in tight turns (reactive)
#define L_MAX             0.9f     // long lookahead on straights (smooth)
// Vector filtering
#define MIN_VECTOR_LEN    15.0f    // ignore very short noise vectors
#define MIN_VECTOR_ANGLE  5.0f     // ignore near-horizontal vectors (deg)

// Low-pass filter alpha ( was hard-coded 0.3/0.7)
// Higher α = more responsive, lower α = smoother
#define LPF_ALPHA 0.7f


float filteredSteering  = 0.0f;
float integralError     = 0.0f;     // UPDATE 1: PID integral term
float lastError         = 0.0f;     // UPDATE 1: PID derivative term
float lastSteeringAngle = SERVO_CENTER;


void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.changeProg("line");
  pixy.setLamp(1, 1);
  servo.attach(19);
  servo.write(SERVO_CENTER);

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
}
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
  normalizeVectors();
  float vx, vy;
  fusedVector(vx, vy);
  normalizeFusedVector(vx, vy);
  float px, py;
  lookaheadPoint(vx, vy, px, py);
  float steering = computeSteering(px, dt); 
  steering = constrain(steering, -20.0f, 20.0f); 
  filteredSteering = LPF_ALPHA * filteredSteering + (1.0f - LPF_ALPHA) * steering;
  float servoAngle = 90 + filteredSteering;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
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
