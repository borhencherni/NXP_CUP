#pragma once

#include <Servo.h>
#include "Config.h"

class ServoController
{
    public:
        ServoController(uint8_t pin = Servo_PIN, int center = SERVO_CENTER, int min = SERVO_MIN, int max = SERVO_MAX) : _pin(pin), _center(center), _min(min), _max(max) {}
        void init(){
            _servo.attach( _pin );
            center();
        };

        float Steer(float steeringangle){
            float angle = constrain(steeringangle, _min, _max);
            _servo.write((int)angle);
            return angle;
        }

        void center(){
            _servo.write(_center);
        }
        
    private:
        Servo _servo;
        uint8_t _pin;
        int _center;
        int _min;
        int _max;
};