#pragma once

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define IN1 6
#define IN2 7
#define IN3 22
#define IN4 23

#define Servo_PIN         17
#define Channel_A_LEFT    2
#define Channel_B_LEFT    3
#define Channel_A_RIGHT   4
#define Channel_B_RIGHT   5

// ─── Pixy2 frame dimensions ───────────────────────────────────────────────────
// The Pixy2 produces 317 px wide images in this configuration.
// frameHeight scales proportionally: 317 / 78 * 51 ≈ 207 px.
// The Pixy2 library exposes pixy.frameWidth / pixy.frameHeight at runtime;
// these defines are only used where a compile-time constant is unavoidable.
#define FRAME_WIDTH       317
#define FRAME_HEIGHT      207

// ─── Pixy2 aspect-ratio correction ───────────────────────────────────────────
// The ratio is derived from the sensor's physical FOV, not its resolution,
// so it stays the same regardless of resolution mode.
#define SCALE_Y(y)        ((y) * 1.53f)

// ─── Vector filtering ─────────────────────────────────────────────────────────
// MIN_VECTOR_LEN scales with frame width: 10 * (317/78) ≈ 40 px.
// Keeps the same physical angular span filtered at the new resolution
#define MIN_VECTOR_LEN    10.0f

// Angle threshold is purely geometric — does not depend on resolution.
#define MIN_TRACK_ANGLE   5.0f

// ─── Lookahead ────────────────────────────────────────────────────────────────
#define L_MIN             0.3f
#define L_MAX             0.7f

// ─── Steering PID gains ───────────────────────────────────────────────────────
#define KP                3.0f
#define KI                0.0f
#define KD                0.0f
#define I_MAX             10.0f

// ─── Servo ────────────────────────────────────────────────────────────────────
#define SERVO_CENTER      90
#define SERVO_MIN         70
#define SERVO_MAX         110

// ─── Low-pass filter ─────────────────────────────────────────────────────────
#define LPF_ALPHA         0.7f

// ─── Speed PID gains ─────────────────────────────────────────────────────────
#define KP_S              2.0f
#define KI_S              0.0f
#define KD_S              0.0f
#define I_MAX_S           15.0f

// ─── Encoder ─────────────────────────────────────────────────────────────────
#define nb_t_p_rot        22
#define SPEED_PERIOD_MS   5

// ─── Steering PID virtua&l-triangle constant ───────────────────────────────────
// STEERING_PIXEL_SCALE converts a pixel error into a steering angle via atan2.
// It must scale with frame width so the same real-world angular error produces
// the same steering output regardless of resolution.
// Previous value was 40 for a 78 px frame:  40 * (317 / 78) ≈ 163
#define STEERING_PIXEL_SCALE  163.0f

// ─── Frame centre (compile-time reference) ────────────────────────────────────
// 317 / 2 = 158.5.  Most code reads pixy.frameWidth/2 dynamically at runtime.
#define SCREEN_CENTER_X       158.5f

// ─── Track geometry ───────────────────────────────────────────────────────────
// Used when only ONE line is visible to estimate the lane centre.
//
// Calibration (do this once on the real track):
//   1. Place the car centred on the track.
//   2. Open Serial monitor; read leftX and rightX.
//   3. Set TRACK_WIDTH_PX = rightX - leftX.
//
// Starting estimate scaled from old value:  45 * (317 / 78) ≈ 183 px.
#define TRACK_WIDTH_PX        215

// ─── Intersection handling ────────────────────────────────────────────────────
// Duration the robot drives straight after a crossing is detected.
// Resolution-independent (time-based).
#define INTERSECTION_HOLD_MS  1500UL

// Continuity filter: max head-position drift between frames to still be
// considered the same line. Scaled: 22 * (317/78) ≈ 89 px.
#define CONTINUITY_THRESHOLD_PX  89.0f

// ─── Adaptive speed ──────────────────────────────────────────────────────────
#define BASE_SPEED           170    // PWM on a straight
#define MIN_SPEED            150    // PWM in the tightest turn
#define INTERSECTION_SPEED   0    // PWM while crossing an intersection


