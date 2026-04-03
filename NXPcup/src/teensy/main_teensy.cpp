#include <Arduino.h>
#include <Pixy2.h>

#include "Config.h"
#include "LineDetector.h"
#include "SteeringController.h"
#include "ServoController.h"
#include "SpeedControl.h"
#include "core_pins.h"

// ─── Hardware objects ─────────────────────────────────────────────────────────
static Pixy2 pixy;
TrackInfo trackInfo;
// ─── Subsystem objects ────────────────────────────────────────────────────────
static LineDetector      lineDetector(pixy);
static SteeringController steering(pixy);
static ServoController   servoCtrl(Servo_PIN);
static SpeedControl      speedCtrl(nb_t_p_rot);

// ─── Timing ───────────────────────────────────────────────────────────────────
static unsigned long lastTime = 0;
// ___ ISR functions ______________________________________________________________________
void ISR_Right() {
    if(digitalRead(Channel_B_RIGHT)==HIGH) speedCtrl.dec_ticks_right();
    else        speedCtrl.inc_ticks_right();
}
void ISR_Left() {
    if(digitalRead(Channel_B_LEFT)==HIGH) speedCtrl.dec_ticks_left();
    else        speedCtrl.inc_ticks_left();
}


// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(Channel_B_RIGHT, INPUT_PULLUP);
    pinMode(Channel_B_LEFT, INPUT_PULLUP);
    pinMode(Channel_A_RIGHT, INPUT_PULLUP);
    pinMode(Channel_A_LEFT, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    attachInterrupt(Channel_A_RIGHT, ISR_Right, CHANGE);
    attachInterrupt(Channel_A_LEFT, ISR_Left, CHANGE);
    

    pixy.init();
    pixy.setLamp(1, 1);
    pixy.changeProg("line");
    delay(1000);
    

    servoCtrl.init();   // attaches servo and moves to centre
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    
    // ── Timing ────────────────────────────────────────────────────────────────
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;   // guard against overflow / first tick
    lastTime = now;

    // ── Line Detection ────────────────────────────────────────────────────────
    if (!lineDetector.update()) {
        //servoCtrl.center();
        //steering.reset();
        lineDetector.getTrackInfo(trackInfo);
    // ── Steering ──────────────────────────────────────────────────────────────
        float steeringAngle = steering.compute(trackInfo, dt);
        float servoAngle    = servoCtrl.Steer(steeringAngle + SERVO_CENTER);
        speedCtrl.runMotors(80, 80);
        digitalWrite(LED_PIN, HIGH);
        //Serial.println("No line detected");
        return;
    }
    digitalWrite(LED_PIN, LOW);
    /*if (trackInfo.isCrossing){
        speedCtrl.runMotors(0, 0);
        servoCtrl.center();
        steering.reset();
        while(true);
    }*/
    //lineDetector.getFusedVector(vx, vy);
     lineDetector.getTrackInfo(trackInfo);
    // ── Steering ──────────────────────────────────────────────────────────────
    float steeringAngle = steering.compute(trackInfo, dt);
    float servoAngle    = servoCtrl.Steer(steeringAngle + SERVO_CENTER);

    // ── Speed Control ────────────────────────────────────────────────────────
   /*float steerRatio = steeringAngle / 20.0f; // normalize to [-1, 1]
   int left_speed = constrain(150 * (1 - steerRatio), 0, 150);
   int right_speed = constrain(150 * (1 + steerRatio), 0, 150);
   speedCtrl.setSpeed(left_speed, right_speed);*/
   speedCtrl.runMotors(100, 100);
}