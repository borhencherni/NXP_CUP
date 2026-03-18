#include <Pixy2.h>
#include <Servo.h>
#define IN1 22
#define IN2 23
#define IN3 14
#define IN4 15

Servo servo;
Pixy2 pixy;
 
void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line");
  servo.attach(19);
  servo.write(90);
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

void loop() {
  pixy.line.getAllFeatures();
  runMotors(100,100);
  servo.write(90);
  delay(1000);
  servo.write(70);
  delay(1000);
  servo.write(110);
  delay(1000);  
}