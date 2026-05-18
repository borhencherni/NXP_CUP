#pragma once

#include <Pixy2.h>
#include "Config.h"
#include "LineDetector.h"

class SteeringController {
public:
    explicit SteeringController(Pixy2& pixy);

    /**
     * Call every loop iteration.
     * @param track  TrackInfo from LineDetector::getTrackInfo()
     * @param dt     time since last call (seconds)
     * @return filtered steering angle in degrees (negative = left, positive = right)
     */
    float compute(TrackInfo& track, float dt);

    /**
     * Enable / disable intersection straight-through mode.
     * While active the controller targets the frame centre so the car drives
     * straight and ignores potentially noisy vector detections.
     */
    void setIntersectionMode(bool active);

    void reset();

private:
    Pixy2& _pixy;
    float  _integralError    = 0.0f;
    float  _lastError        = 0.0f;
    float  _filteredSteering = 0.0f;
    float  _lastcurrentTargetX = 0.0f;
    bool   _inIntersection   = false;   // NEW: intersection straight-drive flag

    float adaptiveLookahead(float vx) const;
    void  lookaheadPoint(float vx, float vy, float& px, float& py) const;
    float currentTarget(const TrackInfo& track) const;
    float computePID(float px, float dt);
};
