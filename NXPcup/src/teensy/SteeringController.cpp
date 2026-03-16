#include "SteeringController.h"


SteeringController::SteeringController(Pixy2& pixy) : _pixy(pixy) {}


// Public 

float SteeringController::compute(float vx, float vy, float dt) {
    float px, py;
    lookaheadPoint(vx, vy, px, py);

    float raw = computePID(px, dt);
    raw = constrain(raw, -20.0f, 20.0f);

    // Low-pass filter to smooth out rapid servo jitter
    _filteredSteering = LPF_ALPHA * _filteredSteering + (1.0f - LPF_ALPHA) * raw;

    return _filteredSteering;
}

void SteeringController::reset() {
    _integralError    = 0.0f;
    _lastError        = 0.0f;
    _filteredSteering = 0.0f;
}

// Private 
/**
 * Adaptive lookahead distance.
 * |vx| ∈ [0, 1] after normalisation: 0 = straight, 1 = sharp turn.
 * More curvature → shorter lookahead (more reactive).
 */
float SteeringController::adaptiveLookahead(float vx) const {
    float curvature = fabsf(vx);
    return L_MAX - curvature * (L_MAX - L_MIN);
}

/** Project the target point onto the frame using the lookahead distance. */
void SteeringController::lookaheadPoint(float vx, float vy, float& px, float& py) const {
    float L = adaptiveLookahead(vx);
    px = _pixy.frameWidth  / 2.0f + vx * L * _pixy.frameWidth;
    py = _pixy.frameHeight         + vy * L * _pixy.frameHeight;
}

/**
 * PID controller.
 * Error is the horizontal deviation of the lookahead point from frame centre.
 * Output is converted from pixels to degrees via atan2.
 */
float SteeringController::computePID(float px, float dt) {
    float error = px - _pixy.frameWidth / 2.0f;

    float p = KP * error;

    _integralError += error * dt;
    _integralError  = constrain(_integralError, -I_MAX, I_MAX);
    float i = KI * _integralError;

    float d = KD * (error - _lastError) / dt;
    _lastError = error;

    float output = p + i + d;
    return atan2f(output, 40.0f) * 180.0f / PI;
}