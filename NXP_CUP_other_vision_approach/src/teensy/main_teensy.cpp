#include <Arduino.h>
#include <Pixy2.h>

#include "Config.h"
#include "LineDetector.h"
#include "SteeringController.h"
#include "ServoController.h"
#include "SpeedControl.h"
#include "core_pins.h"

// ─── Hardware objects ─────────────────────────────────────────────────────────
static Pixy2             pixy;
TrackInfo                trackInfo;

// ─── Subsystem objects ────────────────────────────────────────────────────────
static LineDetector       lineDetector(pixy);
static SteeringController steering(pixy);
static ServoController    servoCtrl(Servo_PIN);
static SpeedControl       speedCtrl(nb_t_p_rot);
int lastNoSeen = 0;
// ─── Timing ───────────────────────────────────────────────────────────────────
static unsigned long lastTime = 0;

// ─── Intersection state ───────────────────────────────────────────────────────
// Once a crossing is detected we keep "intersection mode" active for
// INTERSECTION_HOLD_MS milliseconds so the robot drives straight through the
// entire crossing zone, even if the camera briefly loses the track lines.
static bool          inIntersection       = false;
static unsigned long intersectionStartMs  = 0;

// ─── ISR functions ────────────────────────────────────────────────────────────
void ISR_Right() {
    if (digitalRead(Channel_B_RIGHT) == HIGH) speedCtrl.dec_ticks_right();
    else                                       speedCtrl.inc_ticks_right();
}
void ISR_Left() {
    if (digitalRead(Channel_B_LEFT) == HIGH) speedCtrl.dec_ticks_left();
    else                                      speedCtrl.inc_ticks_left();
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(Channel_B_RIGHT, INPUT_PULLUP);
    pinMode(Channel_B_LEFT,  INPUT_PULLUP);
    pinMode(Channel_A_RIGHT, INPUT_PULLUP);
    pinMode(Channel_A_LEFT,  INPUT_PULLUP);

    attachInterrupt(Channel_A_RIGHT, ISR_Right, CHANGE);
    attachInterrupt(Channel_A_LEFT,  ISR_Left,  CHANGE);
    
    pixy.init();
    pixy.setLamp(1, 1);
    pixy.changeProg("line");
    delay(1000);

    servoCtrl.init();   // attach servo and centre
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    //Serial.println(pixy.frameHeight);
    // ── Timing ────────────────────────────────────────────────────────────────
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
    lastTime = now;

    // ── Line Detection ────────────────────────────────────────────────────────
    bool linesFound = lineDetector.update();
    lineDetector.getTrackInfo(trackInfo);

    if (!linesFound) {
       
        // No vectors at all – hold last steering, stop motors as a safety measure.
        float steeringAngle = 0; //steering.compute(trackInfo, dt);
        servoCtrl.Steer(steeringAngle + SERVO_CENTER );
        speedCtrl.runMotors(100, 100);
        delay(200);
        
        return;
    }

    // ── Intersection state machine ────────────────────────────────────────────
    if (trackInfo.isCrossing && !inIntersection) {
        // Rising edge: just entered an intersection
        inIntersection      = true;
        intersectionStartMs = now;
        steering.setIntersectionMode(true);
        Serial.println(">> Intersection ENTER");
    }

    if (inIntersection && (now - intersectionStartMs >= INTERSECTION_HOLD_MS)) {
        // Timer expired: crossing zone cleared
        inIntersection = false;
        steering.setIntersectionMode(false);
        Serial.println(">> Intersection EXIT");
    }

    // ── Steering ──────────────────────────────────────────────────────────────
    float steeringAngle = steering.compute(trackInfo, dt);
    servoCtrl.Steer(steeringAngle + SERVO_CENTER);

    // ── Adaptive speed ────────────────────────────────────────────────────────
    // Slow down proportionally to the steering angle so the car doesn't
    // overshoot corners, and reduce further when crossing an intersection.
    int targetSpeed;
    if (inIntersection) {
        targetSpeed = INTERSECTION_SPEED;
    } else {
        // Linear reduction: BASE_SPEED at 0°, MIN_SPEED at ±30°
         targetSpeed = BASE_SPEED;
    }

   speedCtrl.runMotors(targetSpeed, targetSpeed);
}
