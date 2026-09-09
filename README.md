# NXPcup — Pixy2-Based Autonomous Line-Following Race Car

An autonomous line-following race car built for the NXP Cup, using a **Pixy2 smart camera** for vision, a **Teensy** microcontroller for all control logic, and a custom lookahead + PID steering pipeline tuned for both straight-line speed and tight cornering.

## How it works — pipeline overview

```
Pixy2 camera (line-tracking mode)
        │  raw line vectors (x0,y0)-(x1,y1)
        ▼
LineDetector          → filters, classifies (left/right/horizontal), and
                         fuses vectors into TrackInfo + a single direction vector
        ▼
SteeringController     → adaptive lookahead → target point (px, py) → PID → steering angle
        ▼
ServoController         → drives the steering servo
        ▼
SpeedControl           → encoder-based closed-loop wheel speed (PID) → motor PWM
```

### 1. Vision — `LineDetector`

The Pixy2 runs in **line-tracking mode** and returns a set of short line-segment vectors per frame. `LineDetector` (`LineDetector.h/.cpp`) processes these every loop:

- **Normalizes** each vector so its "tail" (`y0`) is always the point closest to the car (bottom of frame) and its "head" (`x1,y1`) is the lookahead tip.
- **Filters out noise**: vectors shorter than `MIN_VECTOR_LEN`, and near-horizontal vectors under `MIN_TRACK_ANGLE` are treated separately (see intersections below) rather than as track edges.
- **Classifies** remaining vectors as left-edge or right-edge of the track based on which side of frame-center their base sits on.
- **Sorts and picks the most reliable vector** per side (longest = most trustworthy).
- **Detects intersections**: strongly horizontal vectors are interpreted as a perpendicular crossing line. A **continuity filter** then ignores any left/right candidate that has jumped implausibly far from the last known position (`CONTINUITY_THRESHOLD_PX`), which prevents the crossing line's segments from being mistaken for the actual track edges.
- Also computes a **fused direction vector** `(vx, vy)` — a weighted, normalized combination of all valid vectors (weighted by length × proximity to the car) representing the overall direction of the track ahead.

### 2. Steering — `SteeringController` (lookahead + PID)

Rather than steering toward the nearest point on the line, the car aims at a point **further down the track** — like a driver looking ahead of the car rather than at the front bumper. Full derivation in [`LOOKAHEAD_PIPELINE.md`](LOOKAHEAD_PIPELINE.md); summary:

- **Target X (`currentTarget`)**: if both edges are visible, aim for their midpoint; if only one edge is visible, estimate the far edge using a calibrated `TRACK_WIDTH_PX`; if the car is fully blind, hold the last known target.
- **Adaptive lookahead (`L`)**: interpolates between `L_MIN` (sharp turns → look close, more reactive) and `L_MAX` (straights → look far, smoother) based on curvature (`|vx|`).
- **PID on pixel error**: `error = px - frameCenter` is fed through a P(I)(D) controller (see [`PID_explication.txt`](PID_explication.txt) for the full math). Only `Kp` is currently active — `Ki`/`Kd` are wired up but set to 0.
- **Pixel-to-angle conversion**: the PID's pixel-space output is converted to a steering angle via `atan2f(output, STEERING_PIXEL_SCALE)`, which gives *soft saturation* — the angle asymptotically approaches its bound instead of growing unbounded — on top of the hard `constrain()` clamp.
- **Low-pass filtering** (`LPF_ALPHA`) smooths the final steering angle between frames to avoid jittery servo motion.
- **Intersection mode**: while a crossing is being driven through, steering targets frame-center directly (drive straight) rather than trusting potentially noisy vector detections, for a fixed hold duration (`INTERSECTION_HOLD_MS`).

### 3. Speed control — `SpeedControl`

Each wheel has a quadrature encoder; interrupt service routines (`ISR_Left`/`ISR_Right` in `main_teensy.cpp`) increment/decrement tick counters based on direction. `SpeedControl` converts ticks into RPM every `SPEED_PERIOD_MS`, then runs an independent PID per wheel (`KP_S`/`KI_S`/`KD_S`) to hold a target PWM/speed and drives the motor driver pins directly.

### 4. Actuation

- **`ServoController`** — thin wrapper around the Arduino `Servo` library; clamps steering angle to `[SERVO_MIN, SERVO_MAX]` around `SERVO_CENTER`.
- **`motor.h`/`motor.cpp`** — lower-level H-bridge motor driver helpers (`Motor_Init`, `Motor_SetSpeed`, `Motor_Stop`, `Motor_SetSteering`).

### 5. Vision helpers

- **`vision.h`/`vision.cpp`** — standalone Pixy2 helper module (`Vision_Init`, `Vision_GetAngleError`, `Vision_PrintFeatures`) used for early prototyping/debugging of line angle detection, independent of the main `LineDetector` pipeline used in the final car.

## Configuration

All tunable constants live in [`Config.h`](Config.h): pin mapping, Pixy2 frame dimensions and aspect-ratio correction, vector filtering thresholds, lookahead range, steering/speed PID gains, servo limits, low-pass filter strength, encoder ticks-per-rotation, intersection timing, and adaptive-speed PWM values. Several constants are explicitly scaled from an earlier, smaller Pixy2 resolution (78×51 px) to the current one (317×207 px) — see the inline comments in `Config.h` for the scaling rationale before changing camera resolution.

## Hardware

- **Teensy** microcontroller running the full control loop (`src/teensy/`)
- **Pixy2** smart camera, in line-tracking mode
- 2× DC motors with quadrature encoders (differential drive)
- Steering servo (Ackermann-style steering, not differential steering)
- Custom chassis (see `Mécanique/` — SolidWorks CAD, incl. 3D-printed parts and a Pixy2 camera mount)

> An ESP32 target (`src/esp/main_esp.cpp`) exists in this repo but was not developed further — all working logic is in the Teensy code under `src/teensy/`.

## Repo Structure

```
NXPcup/
├── src/
│   └── teensy/
│       ├── main_teensy.cpp         # main control loop
│       ├── Config.h                # all tunables
│       ├── LineDetector.h/.cpp     # vision processing
│       ├── SteeringController.h/.cpp
│       ├── SpeedControl.h/.cpp
│       └── ServoController.h
├── lib/
│   ├── motor.h/.cpp                 # low-level motor driver
│   ├── vision.h/.cpp                # standalone Pixy2 helper (prototyping)
│   └── Pixy2/                      # vendored Pixy2 camera library
├── LOOKAHEAD_PIPELINE.md           # full lookahead math writeup
├── PID_explication.txt             # full PID + angle-conversion writeup
└── platformio.ini
```

## Further Reading

- [`LOOKAHEAD_PIPELINE.md`](LOOKAHEAD_PIPELINE.md) — step-by-step derivation of how a direction vector becomes a lookahead target point
- [`PID_explication.txt`](PID_explication.txt) — full breakdown of the PID controller and the pixel-to-degrees conversion, with a worked numeric example
