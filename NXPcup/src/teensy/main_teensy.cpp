#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#define IN1 22
#define IN2 23
#define IN3 6
#define IN4 7
#define ECHO 19
#define TRIG 18

Pixy2 pixy;
Servo servo;
// Aspect ratio correction: Pixy2 pixels are not square
// frameWidth=78, frameHeight=51 → real ratio ≈ 1.53
//frameWidth  = 78 pixels   →  covers the full horizontal FOV
//frameHeight = 51 pixels   →  covers the full vertical FOV
//ratio = frameWidth / frameHeight = 78 / 51 ≈ 1.53
//This means 1 pixel of height = 1.53 pixels of width in real-world distance.
#define SCALE_Y(y)        ((y) * 1.53f)
#define KP                5.0f
#define KI                0.3f    
#define KD                1.0f 
#define SERVO_CENTER      90
#define SERVO_MIN         50      
#define SERVO_MAX         140 
#define I_MAX             10.0f   // Integral windup clamp
// Lookahead 
#define L_MIN             0.15f     // short lookahead in tight turns (reactive)
#define L_MAX             0.75f     // long lookahead on straights (smooth)
// Vector filtering
#define MIN_VECTOR_LEN    8.0f    // ignore very short noise vectors
#define MAX_VECTOR_LEN    110.0f   // cap max vector length to avoid outliers dominating
#define MIN_VECTOR_ANGLE  2.0f     // ignore near-horizontal vectors (deg)

#define STEERING_DEADBAND  2.0f   // degrees — ignore corrections smaller than this
#define MAX_SERVO_STEP    10.0f    // max degrees servo can move per cycle

#define CONTROL_PERIOD_MS 5

#define MOTOR_SPEED 230
#define SERVO_FILTER_SIZE 5

float servoBuffer[SERVO_FILTER_SIZE];
int servoIndex = 0;
bool bufferFilled = false;

float filteredSteering  = 0.0f;
float integralError     = 0.0f;     //  PID integral term
float lastError         = 0.0f;     //  PID derivative term
float lastSteeringAngle = SERVO_CENTER ; // Start slightly left to encourage initial turn onto line
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line");
  servo.attach(17);
  //servo.write(SERVO_CENTER);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  startTime = millis();
}

static void driveMotor(int forwardPin, int reversePin, int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0)
  {
    analogWrite(forwardPin, speed);
    analogWrite(reversePin, 0);
  }
  else
  {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -speed);
  }
}

void runMotors(int LPWM,int RPWM){
  driveMotor(IN1, IN2, LPWM);
  driveMotor(IN3, IN4, RPWM);
}

float readUltrasonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  float distance = (duration * 0.0343) / 2; // Convert to cm
  return distance;
}

float computeMedian(float *arr, int size)
{
  float temp[SERVO_FILTER_SIZE];
  
  // Copy array
  for(int i = 0; i < size; i++)
    temp[i] = arr[i];

  // Simple bubble sort
  for(int i = 0; i < size-1; i++)
  {
    for(int j = i+1; j < size; j++)
    {
      if(temp[j] < temp[i])
      {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[size/2]; // middle value
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

  float length = sqrtf(dx*dx + dy*dy);

  float proximity = pixy.line.vectors[i].m_y0 / (float)pixy.frameHeight;

  return length * proximity;
}
bool validVector(int i)
{
  float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
  float dy = SCALE_Y(pixy.line.vectors[i].m_y1) - SCALE_Y(pixy.line.vectors[i].m_y0);

  float length = sqrtf(dx*dx + dy*dy);

  if(length < MIN_VECTOR_LEN)
    return false;
  if(length > MAX_VECTOR_LEN)
    return false;
  float angle = atan2f(dy,dx) * 180.0f / PI;

  if(fabsf(angle) < MIN_VECTOR_ANGLE)
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
  }

  if(validCount == 0)
  {
    vx = 0.0f;
    vy = 1.0f; // default forward+
  }
}
void normalizeFusedVector(float &vx, float &vy)
{
  float mag = sqrtf(vx*vx + vy*vy);
  if(!isfinite(mag) || mag < 1e-6f)
  {
    vx = 0.0f;
    vy = 1.0f;   // default forward vector
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
  if (lastTime != 0 && (now - lastTime) < CONTROL_PERIOD_MS)
    return;

  float dt = (lastTime == 0) ? (CONTROL_PERIOD_MS / 1000.0f) : ((now - lastTime) / 1000.0f);
  if (dt <= 0.0f || dt > 0.5f) dt = CONTROL_PERIOD_MS / 1000.0f;
  lastTime = now;

  pixy.line.getAllFeatures();

  normalizeVectors();
  float vx, vy;
  fusedVector(vx, vy); 
  normalizeFusedVector(vx, vy);
  //Serial.print("Fused vector: vx="); 
  //Serial.print(vx); 
  //Serial.print(" vy="); 
  //Serial.println(vy);
  //Serial.print("Num vectors: ");
  //Serial.println(pixy.line.numVectors);
  float px, py;
  lookaheadPoint(vx, vy, px, py);
  float steering = computeSteering(px, dt);
  steering = constrain(steering, -30.0f, 30.0f);

  // Deadband
  if (fabsf(steering) < STEERING_DEADBAND)
      steering = 0.0f;

  // Slew rate limit
  float servoAngle = SERVO_CENTER + steering;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  float step = constrain(servoAngle - lastSteeringAngle, -MAX_SERVO_STEP, MAX_SERVO_STEP);
  servoAngle = lastSteeringAngle + step;
  lastSteeringAngle = servoAngle;

  servoBuffer[servoIndex] = servoAngle;
  servoIndex++;

  if(servoIndex >= SERVO_FILTER_SIZE)
  {
    servoIndex = 0;
    bufferFilled = true;
  }

  // Use filtered value only when buffer is ready
  float filteredServo = servoAngle;

  if(bufferFilled)
  {
    filteredServo = computeMedian(servoBuffer, SERVO_FILTER_SIZE);
  }

  // Apply servo
  servo.write((int)filteredServo);
  runMotors(MOTOR_SPEED, MOTOR_SPEED);
  if(millis()-startTime>10000){
    float distance = readUltrasonic();
    if (distance > 15 && distance < 30){
      runMotors(0, 0);
      while(1);
    }
  }
}
