#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#define IN1 22
#define IN2 23
#define IN3 14
#define IN4 15

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
#define SERVO_MAX         100  
#define I_MAX             15.0f   // Integral windup clamp
// Lookahead 
#define L_MIN             0.4f     // short lookahead in tight turns (reactive)
#define L_MAX             0.9f     // long lookahead on straights (smooth)
// Vector filtering
#define MIN_VECTOR_LEN    5.0f    // ignore very short noise vectors
#define MIN_VECTOR_ANGLE  2.0f     // ignore near-horizontal vectors (deg)

// Low-pass filter alpha 
#define LPF_ALPHA 0.7f

#define MOTOR_SPEED 150

float filteredSteering  = 0.0f;
float integralError     = 0.0f;     //  PID integral term
float lastError         = 0.0f;     //  PID derivative term
float lastSteeringAngle = SERVO_CENTER;


void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line");
  servo.attach(19);
  servo.write(SERVO_CENTER);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void runMotors(int LPWM,int RPWM){
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);
  if (LPWM > 0){
    analogWrite(IN1, LPWM);
    analogWrite(IN2, 0);
  }
  else{
    analogWrite(IN1, 0);
    analogWrite(IN2, LPWM);
  }
  if (RPWM > 0){
    analogWrite(IN3, RPWM);
    analogWrite(IN4, 0);
  }
  else{
    analogWrite(IN3, 0);
    analogWrite(IN4, RPWM);
  }
  
}

void sendDataToESP(float vx, float vy, float steeringangle, float servoangle)
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
  int validCount = 0;

  for(int i=0;i<pixy.line.numVectors;i++)
  {
    if(!validVector(i))
    continue;
    validCount++;
    float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
    float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

    float w = vectorWeight(i);

    vx += dx * w;
    vy += dy * w;
    if(validCount == 0)
  {
    vx = 0;
    vy = 1; // default forward+
  }
  }
}
void normalizeFusedVector(float &vx, float &vy)
{
  float mag = sqrt(vx*vx + vy*vy);
  if(!isfinite(mag) || mag < 1e-6f)
  {
    vx = 0;
    vy = 1;   // default forward vector
    return;
  }
    vx /= mag;
    vy /= mag;
  
}
float adaptiveLookahead(float vx) {
    // |vx| ∈ [0,1] after normalization: 0=straight, 1=sharp turn
    float curvature = fabsf(vx);
    // Linearly interpolate: more curvature → shorter lookahead
    return L_MAX - curvature * (L_MAX - L_MIN);
}

void lookaheadPoint(float vx, float vy, float& px, float& py) {
    float L = adaptiveLookahead(vx);   
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
  if(!isfinite(vx) || !isfinite(vy) || (vx == 0 && vy == 0))
{
    Serial.println("Invalid fused vector");
    servo.write(SERVO_CENTER);
    return;
}
  normalizeFusedVector(vx, vy);
  Serial.print("Fused vector: vx="); 
  Serial.print(vx); 
  Serial.print(" vy="); 
  Serial.println(vy);
  Serial.print("Num vectors: ");
  Serial.println(pixy.line.numVectors);
  float px, py;
  lookaheadPoint(vx, vy, px, py);
  float steering = computeSteering(px, dt); 
  steering = constrain(steering, -20.0f, 20.0f); 
  filteredSteering = LPF_ALPHA * filteredSteering + (1.0f - LPF_ALPHA) * steering;
  float servoAngle = 90 + filteredSteering;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  servo.write(servoAngle);
  runMotors(MOTOR_SPEED, MOTOR_SPEED);
  //sendDataToESP(vx, vy, filteredSteering, servoAngle);
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
