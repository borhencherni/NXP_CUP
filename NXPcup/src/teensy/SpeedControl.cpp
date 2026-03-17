#include "SpeedControl.h"
#include "Config.h"
#include <Arduino.h>
//public
void SpeedControl::setSpeed(int L_speed, int R_speed){
    unsigned long now = millis();

    if (_previous_time == 0) { _previous_time = now; return; }
    if (now - _previous_time < SPEED_PERIOD_MS) return;

    float dt = (now - _previous_time) / 1000.0f;
    _previous_time = now;

    _target_speed_left = L_speed;
    _target_speed_right = R_speed;

    compute_current_speed(dt);
    float PID_speed_left = compute_PID_speed(_current_speed_left, _target_speed_left,_integralErrorLeft, _lastErrorLeft , dt);
    float PID_speed_right = compute_PID_speed(_current_speed_right, _target_speed_right, _integralErrorRight, _lastErrorRight , dt);
    runMotors(constrain(PID_speed_left,0,255),constrain(PID_speed_right,0,255));
}

// private
void SpeedControl::compute_current_speed(float dt){
    
    noInterrupts();
    int tl = _ticks_left;  _ticks_left = 0;
    int tr = _ticks_right; _ticks_right = 0;
    interrupts();
    _current_speed_left = ((float)tl/ _nb_ticks_PR)*(60.0f)/ (dt);
    _current_speed_right = ((float)tr / _nb_ticks_PR)*(60.0f)/ (dt); 
   
}

float SpeedControl::compute_PID_speed(float current_speed, float target_speed,float& IntegralError, float& lastError, float dt){
    float Kp = KP_S;
    float Ki = KI_S;
    float Kd = KD_S;
    float error = target_speed - current_speed;
    IntegralError = constrain(IntegralError + (error * dt), -I_MAX_S, I_MAX_S);
    float derivativeError = error - lastError;
    float output = Kp * error + Ki * IntegralError + Kd * (derivativeError/dt);
    lastError = error;
    return output;
}

void SpeedControl::runMotors(int left_PWM, int right_PWM){
    if(left_PWM < 0){
        analogWrite(IN1, 0);
        analogWrite(IN2, -left_PWM);
    }
    else{
        analogWrite(IN1, left_PWM);
        analogWrite(IN2, 0);
    }
    if(right_PWM < 0){
        analogWrite(IN3, 0);
        analogWrite(IN4, -right_PWM);
    }
    else{
        analogWrite(IN3, right_PWM);
        analogWrite(IN4, 0);
    }
    
}