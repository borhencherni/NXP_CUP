#pragma once

#include "Config.h"


class SpeedControl{
    public:
        explicit SpeedControl( int nb_ticks_PR = nb_t_p_rot): _nb_ticks_PR(nb_ticks_PR) {};
        void setSpeed(int L_speed, int R_speed);
        int inc_ticks_left(){return _ticks_left++;};
        int inc_ticks_right(){return _ticks_right++;};
        int dec_ticks_left(){return _ticks_left--;};
        int dec_ticks_right(){return _ticks_right--;};
        void runMotors(int left_PWM, int right_PWM);

    private:
        void compute_current_speed(float dt);
        float compute_PID_speed(float current_speed, float target_speed,float& IntegralError, float& lastError, float dt);
        

        unsigned long _previous_time = 0;
        int _nb_ticks_PR;
        volatile int _ticks_left = 0;
        volatile int _ticks_right = 0;
       
        float _target_speed_left;
        float _target_speed_right;
        float _current_speed_left = 0;
        float _current_speed_right = 0;

        float _integralErrorLeft = 0.0f;
        float _integralErrorRight = 0.0f;
        float _lastErrorLeft = 0.0f;
        float _lastErrorRight = 0.0f;
        
};