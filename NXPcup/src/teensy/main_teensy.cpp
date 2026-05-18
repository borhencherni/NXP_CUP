
#include <Arduino.h>

#include "teensy_config.h"
#include "teensy_motors.h"
#include "teensy_sensors.h"
#include "teensy_state.h"
#include "teensy_vision.h"

static String espCommandBuffer;

static void sendTelemetry(float vx, float vy, float steeringAngle, float servoAngle)
{
  Serial2.print("D,");
  Serial2.print(vx, 4);
  Serial2.print(',');
  Serial2.print(vy, 4);
  Serial2.print(',');
  Serial2.print(steeringAngle, 2);
  Serial2.print(',');
  Serial2.println(servoAngle, 2);
}

static void applyManualCommand(char command)
{
  switch (command)
  {
    case 'F':
      runMotors(120, 120);
      break;
    case 'B':
      runMotors(-120, -120);
      break;
    case 'L':
      runMotors(90, 120);
      break;
    case 'R':
      runMotors(120, 90);
      break;
    case 'S':
    default:
      runMotors(0, 0);
      break;
  }
}

static void processEspCommands()
{
  while (Serial2.available())
  {
    char c = Serial2.read();

    if (c == '\n')
    {
      espCommandBuffer.trim();
      if (espCommandBuffer.startsWith("C,"))
      {
        char command = espCommandBuffer.charAt(2);
        manualCommand = command;
        manualCommandUntil = millis() + 300;
        if (command == 'S')
        {
          manualCommandUntil = millis();
        }
      }
      espCommandBuffer = "";
    }
    else
    {
      espCommandBuffer += c;
    }
  }
}

static char manualCommand = 'S';
static unsigned long manualCommandUntil = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Vision_Init();
  Motors_Init();
  Sensors_Init();
  startTime = millis();
}

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
  float px, py;
  lookaheadPoint(vx, vy, px, py);
  float steering = computeSteering(px, dt);
  detectLaneSides();

  // Step 2: update dash detection
  /*updateDashDetection();

  // Step 3: check finish line
  if (detectFinishLine())
  {
    runMotors(0, 0);
    servo.write(SERVO_CENTER);

    Serial.println("FINISH LINE DETECTED");

    while (1);
  }*/
  float laneCorrection = 0.0f;
  float strength = fabsf(vx); // curvature factor

  if(hasRight && !hasLeft)
  {
    laneCorrection = -LEFT_GAIN * (0.5f + strength);
  }
  else if(hasLeft && !hasRight)
  {
    laneCorrection = RIGHT_GAIN * (0.5f + strength);
  }

  // Apply correction ONCE
  steering += laneCorrection;

  // THEN constrain
  steering = constrain(steering, -40.0f, 40.0f);
  if (fabsf(steering) < STEERING_DEADBAND)
      steering = 0.0f;

  if(fabsf(vx) < 0.05f && vy > 0.9f)
  {
  steering = 0.0f;
  }
  float servoAngle = SERVO_CENTER + steering;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  float step = constrain(servoAngle - lastSteeringAngle, -MAX_SERVO_STEP, MAX_SERVO_STEP);
  servoAngle = lastSteeringAngle + step;
  lastSteeringAngle = servoAngle;
  Serial.print("Steering: ");
  Serial.print(steering); 
  Serial.print(" | Servo Angle: ");
  Serial.println(servoAngle);

  servoBuffer[servoIndex] = servoAngle;
  servoIndex++;

  if(servoIndex >= SERVO_FILTER_SIZE)
  {
    servoIndex = 0;
    bufferFilled = true;
  }

  // Use filtered value only when buffer is ready
  float filteredServo = servoAngle;

  if(bufferFilled && !isSharpTurn(steering))
  {
  filteredServo = computeMedian(servoBuffer, SERVO_FILTER_SIZE);
  }
  servo.write((int)filteredServo);
  float turnFactor = fabsf(steering) / 50.0f; // normalize 0 → 1

  int speed = MOTOR_SPEED * (1.0f - 0.6f * turnFactor); 

  speed = constrain(speed, 180, MOTOR_SPEED);

  processEspCommands();
  if (manualCommandUntil > millis())
  {
    applyManualCommand(manualCommand);
  }
  else
  {
    runMotors(120, 120);
  }

  sendTelemetry(vx, vy, steering, servoAngle);

  if(millis()-startTime>13000){
    speed=110;
    runMotors(120, 120);
    float distance = readUltrasonic();
    if (distance > 15 && distance < 30){
      runMotors(0, 0);
      while(1);
    }
  }
}
