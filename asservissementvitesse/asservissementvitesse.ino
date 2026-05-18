#include <Arduino.h>
#define PI 3.14159265358979323846
#define IN1 5
#define IN2 6
#define IN3 9
#define IN4 10
#define interruptPinRA 2
#define interruptPinRB 3
#define interruptPinLA 18
#define interruptPinLB 19


const float PWM_max = 255.0f;

float kpS_R = 0.8f;
float kiS_R = 0.2f;
float kdS_R = 0.01f;

float kpS_L = 0.8f;
float kiS_L = 0.2f;
float kdS_L = 0.01f;

float PWM_offset_R = 0.0f;
float PWM_offset_L = 0.0f;


// Target & measured speeds (mm/s)
float target_speed_R = 0.0f;
float target_speed_L = 0.0f;

float current_speed_R = 0.0f;
float current_speed_L = 0.0f;

float speed_integral_R = 0.0f;
float speed_integral_L = 0.0f;

float speed_derivative_R = 0.0f;
float speed_derivative_L = 0.0f;

float previous_speed_error_R = 0.0f;
float previous_speed_error_L = 0.0f;

float PWM_R = 0.0f;
float PWM_L = 0.0f;

unsigned long previous_speed_control_time = 0;

volatile long encoder_ticks_R = 620;
volatile long encoder_ticks_L = 620;

const float WHEEL_DIAMETER_MM = 65.0f;     
const int   TICKS_PER_REV     = 620;       

const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;
 
const float MM_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;


void resetSpeedPID()
{
    speed_integral_R = 0.0f;
    speed_integral_L = 0.0f;

    speed_derivative_R = 0.0f;
    speed_derivative_L = 0.0f;

    previous_speed_error_R = 0.0f;
    previous_speed_error_L = 0.0f;

    PWM_R = 0.0f;
    PWM_L = 0.0f;

    previous_speed_control_time = micros();
}

void interruptR() {
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) {
    encoder_ticks_R--;
  } else {
    encoder_ticks_R++;
  }
}

void interruptL() {
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) {
    encoder_ticks_L++;
  } else {
    encoder_ticks_L--;
  }
}



void setup()
{
    Serial.begin(115200);
    resetSpeedPID();
    pinMode(interruptPinRA, INPUT_PULLUP);
    pinMode(interruptPinRB, INPUT_PULLUP);
    pinMode(interruptPinLA, INPUT_PULLUP);
    pinMode(interruptPinLB, INPUT_PULLUP);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);
}


void loop()
{
    unsigned long now = micros();
    float dt = (now - previous_speed_control_time) * 1e-6f;

    target_speed_R = 200.0f; // mm/s
    target_speed_L = 200.0f;

    long ticks_R, ticks_L;
    noInterrupts();
    ticks_R = encoder_ticks_R;
    ticks_L = encoder_ticks_L;
    encoder_ticks_R = 0;
    encoder_ticks_L = 0;
    interrupts();

    float measured_speed_R = (ticks_R * MM_PER_TICK) / dt;
    float measured_speed_L = (ticks_L * MM_PER_TICK) / dt;

    current_speed_R = 0.8f * current_speed_R + 0.2f * measured_speed_R;
    current_speed_L = 0.8f * current_speed_L + 0.2f * measured_speed_L;




    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.001f;


    float error = target_speed_R - current_speed_R;

    speed_derivative_R =
        speed_derivative_R * 0.999f +
        0.001f * (error - previous_speed_error_R) / dt;

    speed_integral_R += error * dt;

    previous_speed_error_R = error;

    PWM_R =
        PWM_offset_R +
        kpS_R * error +
        kdS_R * speed_derivative_R +
        kiS_R * speed_integral_R;

    PWM_R = (PWM_R > PWM_max) ? PWM_max :
            (PWM_R < -PWM_max) ? -PWM_max : PWM_R;

    PWM_R = (target_speed_R == 0.0f) ? 0.0f : PWM_R;

    
    if (PWM_R == PWM_max || PWM_R == -PWM_max)
        speed_integral_R -= 2.0f * error * dt;


    error = target_speed_L - current_speed_L;

    speed_derivative_L =
        speed_derivative_L * 0.999f +
        0.001f * (error - previous_speed_error_L) / dt;

    speed_integral_L += error * dt;

    previous_speed_error_L = error;
    previous_speed_control_time = now;

    PWM_L =
        PWM_L * 0.95f +
        0.05f * (
            PWM_offset_L +
            kpS_L * error +
            kdS_L * speed_derivative_L +
            kiS_L * speed_integral_L
        );

    PWM_L = (PWM_L > PWM_max) ? PWM_max :
            (PWM_L < -PWM_max) ? -PWM_max : PWM_L;

    PWM_L = (target_speed_L == 0.0f) ? 0.0f : PWM_L;

    if (PWM_L == PWM_max || PWM_L == -PWM_max)
        speed_integral_L -= 2.0f * error * dt;



    Serial.print("PWM_R: ");
    Serial.print(PWM_R);
    Serial.print(" | PWM_L: ");
    Serial.println(PWM_L);

    delay(5); 
}
