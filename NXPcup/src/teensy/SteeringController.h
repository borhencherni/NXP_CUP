#pragma once

#include <Pixy2.h> 
#include "Config.h"
#include "LineDetector.h"


class SteeringController {
        /*fel class hethi bech nekhthou fused vecteur normalisé elli lguineh 
        w bech n7awlouh l steering angle. kifech bech na3mlou:
        1- adaptivelookahead: ta3tina point elli n7abou nemchoulha (target)(px,py)
        2-PID controller howa elli bech n7awlou bih l steering angle
        3-nconstrainouh l steering angle bin 20 w -20*/



public:
    explicit SteeringController(Pixy2& pixy);
    /**
     - Call every loop iteration.
     - param vx   normalised horizontal component of fused line vector
     - param vy   normalised vertical   component of fused line vector
     - param dt   time since last call (seconds)
     - return     filtered steering angle in degrees (negative = left, positive = right)
     */
    float compute(TrackInfo& track, float dt);
    void reset();

private:
    Pixy2& _pixy;
    float _integralError     = 0.0f;     //  PID integral term
    float _lastError         = 0.0f;     //  PID derivative term
    float _filteredSteering  = 0.0f;
    float _lastcurrentTargetX = 0.0f;    

    float adaptiveLookahead(float vx) const;
    void lookaheadPoint(float vx, float vy, float& px, float& py) const;
    float currentTarget(TrackInfo& track) const;
    float computePID(float px, float dt);
};