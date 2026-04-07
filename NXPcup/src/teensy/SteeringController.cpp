#include "SteeringController.h"
#include "LineDetector.h"

SteeringController::SteeringController(Pixy2& pixy) : _pixy(pixy) {}

// ─── Public ───────────────────────────────────────────────────────────────────

float SteeringController::compute(TrackInfo& track, float dt) {
    float px;

    if (_inIntersection) {
        // Drive straight through the crossing: target the frame centre.
        // The PID will smoothly bring the car back to centre if it drifted.
        px = _pixy.frameWidth / 2.0f;
    } else {
        px = currentTarget(track);
        _lastcurrentTargetX = px;   // only update memory when NOT in intersection
    }

    float raw = computePID(px, dt);

    // FIX: was constrain(-30, 20) — asymmetric limit clipped right turns to
    // only 20° while allowing left turns to 30°.  Now symmetric ±30°.
    raw = constrain(raw, -30.0f, 30.0f);

    _filteredSteering = LPF_ALPHA * _filteredSteering + (1.0f - LPF_ALPHA) * raw;

    return constrain(_filteredSteering, -30.0f, 30.0f);
}

void SteeringController::setIntersectionMode(bool active) {
    _inIntersection = active;
    if (active) {
        // Reset integral to avoid a sudden jump when we re-enter normal mode
        _integralError = 0.0f;
    }
}

void SteeringController::reset() {
    _integralError    = 0.0f;
    _lastError        = 0.0f;
    _filteredSteering = 0.0f;
}

// ─── Private ──────────────────────────────────────────────────────────────────

float SteeringController::currentTarget(const TrackInfo& track) const {
    float frameCenter = _pixy.frameWidth / 2.0f;   // 39 px

    if (track.hasLeft && track.hasRight) {
        // Case A: both lines visible – geometric centre
        return ((track.leftX + track.rightX) / 2.0f)-10;
    }
    else if (track.hasLeft) {
        // Case B: right line lost – estimate centre from left line.
        // TRACK_WIDTH_PX is now 45 (was 237 — a 3× overestimate that put the
        // target completely off-screen and caused violent overcorrection).
        return track.leftX + (TRACK_WIDTH_PX / 2.0f) -10;
    }
    else if (track.hasRight) {
        // Case C: left line lost
        return track.rightX - (TRACK_WIDTH_PX / 2.0f) - 10;
    }
    else {
        // Case D: fully blind → hold last known target
        return _lastcurrentTargetX;
    }
}

float SteeringController::adaptiveLookahead(float vx) const {
    float curvature = fabsf(vx);
    return L_MAX - curvature * (L_MAX - L_MIN);
}

void SteeringController::lookaheadPoint(float vx, float vy,
                                         float& px, float& py) const {
    float L = adaptiveLookahead(vx);
    px = _pixy.frameWidth  / 2.0f + vx * L * _pixy.frameWidth;
    py = _pixy.frameHeight         + vy * L * _pixy.frameHeight;
}

float SteeringController::computePID(float px, float dt) {
    float error = px - (_pixy.frameWidth / 2.0f);

    float p = KP * error;

    _integralError += error * dt;
    _integralError  = constrain(_integralError, -I_MAX, I_MAX);
    float i = KI * _integralError;

    float d = KD * (error - _lastError) / dt;
    _lastError = error;

    float output = p + i + d;
    // Convert pixel error to degrees through the virtual-triangle constant
    return atan2f(output, STEERING_PIXEL_SCALE) * 180.0f / PI;
}
