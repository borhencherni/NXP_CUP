# From (vx, vy) to (px, py) — Lookahead Pipeline

## Context

The Pixy2 camera sees the line on the track and returns a set of vectors.
After filtering, fusing, and normalizing those vectors, we end up with a
single **unit vector (vx, vy)** that represents the direction of the line
ahead of the car.

The goal of this pipeline is to convert that direction into a **target point
(px, py)** in the camera frame — a point that the car should aim for.

---

## Step 1 — What is (vx, vy) exactly?

After `normalizeFusedVector()`, the vector (vx, vy) is a **unit vector**:

```
√(vx² + vy²) = 1   always
```

This means the length is always exactly 1. Normalization throws away the
length information and keeps only the **direction** of the line.

### What the values mean

```
vx =  0.0   → line goes perfectly straight up     → no horizontal component
vx =  1.0   → line goes perfectly horizontal right → maximum right turn
vx = -1.0   → line goes perfectly horizontal left  → maximum left turn

vy = -1.0   → line goes straight up (away from car) → normal forward direction
vy =  0.0   → line is perfectly horizontal           → extreme case
```

> In Pixy2 coordinates, y increases downward. So a line going away from
> the car (upward in the frame) has a negative vy after normalization.

### Geometric interpretation

Since (vx, vy) is a unit vector, it can be written as:
```
vx = cos(θ)
vy = sin(θ)
```
where θ is the angle of the line relative to the horizontal axis.

If you walk **1 unit** in the direction (vx, vy), you move exactly:
- `vx` units horizontally
- `vy` units vertically

---

## Step 2 — What is the lookahead distance L?

Instead of steering toward the nearest point on the line, we look **ahead**
by a distance L along the direction vector — like a driver looking further
down the road rather than at the front bumper.

L is a **fraction of the frame size** — a number between 0 and 1 that
represents what percentage of the frame we want to look ahead.

```
L = 0.4  →  look 40% of the frame ahead  (close, reactive)
L = 0.9  →  look 90% of the frame ahead  (far, smooth)
```

In actual pixels:
```
L × frameWidth  = L × 78  pixels of horizontal reach
L × frameHeight = L × 51  pixels of vertical   reach
```

---

## Step 3 — Adaptive lookahead (computing L)

L is not fixed — it adapts based on how curved the road is.

### Why adaptive?

- On a **straight line** → look far ahead (smooth, stable steering)
- On a **sharp turn**   → look close   (reactive, don't overshoot the corner)

### Computing curvature

```cpp
float curvature = fabsf(vx);
```

Since vx ∈ [-1, 1] after normalization:
- `vx = 0`  → line going straight → curvature = 0
- `vx = ±1` → line going sideways → curvature = 1

`fabsf` takes the absolute value because we only care about *how much*
curvature there is, not which direction (left or right).

### The interpolation formula

```cpp
L = L_MAX - curvature * (L_MAX - L_MIN);
```

Let's break this down. First compute the constant part:
```
L_MAX - L_MIN = 0.9 - 0.4 = 0.5
```

So the formula simplifies to:
```
L = 0.9 - curvature × 0.5
```

| curvature |             L            |         meaning         |
|-----------|--------------------------|-------------------------|
| 0.0       | 0.9 - 0×0.5 = **0.9**    | straight → look far     |
| 0.5       | 0.9 - 0.5×0.5 = **0.65** | medium turn             |
| 1.0       | 0.9 - 1×0.5 = **0.4**    | sharp turn → look close |

As curvature increases from 0 to 1, L decreases linearly from 0.9 to 0.4.
This is called **linear interpolation** — the output changes at a constant
rate as the input changes.

Visually:
```
curvature:  0 ──────────── 0.5 ──────────── 1
         L=0.9             0.65          L=0.4
         (far)                          (close)
```

---

## Step 4 — Computing the pixel displacement (step_x, step_y)

Now that we have the direction (vx, vy) and the lookahead distance L,
we compute how many pixels to move from the car's position.

```
step_x = vx × L × frameWidth
step_y = vy × L × frameHeight
```

### Reading the formula

Read it as two multiplications in sequence:

```
L × frameWidth  →  total pixels we want to travel
                   e.g. 0.75 × 78 = 58.5 pixels total

× vx            →  of those 58.5 pixels, how many go horizontally
                   e.g. × 0.3 = 17.5 pixels to the right
```

The same logic applies for step_y with frameHeight.

### Why multiply x and y by different frame dimensions?

The frame is **not square** (78 × 51 pixels). If we used the same scale
for both axes, a vector pointing at 45° in real life would appear at a
wrong angle in pixel space.

By multiplying x by `frameWidth` and y by `frameHeight` separately, we
correctly stretch the unit vector into the actual pixel coordinate space
of the camera frame.

Example with a 45° line:
```
vx = 0.707,  vy = -0.707   (45° diagonal, unit vector)
L  = 0.75

step_x = 0.707 × 0.75 × 78 = 41.4 pixels right
step_y = 0.707 × 0.75 × 51 = 27.1 pixels up
```

Notice step_x ≠ step_y even though vx = vy in magnitude — because 78 ≠ 51.

---

## Step 5 — Computing the target point (px, py)

The car is always at the **bottom center** of the frame:
```
car_x = frameWidth  / 2 = 78 / 2 = 39   (horizontal center)
car_y = frameHeight      = 51            (bottom of frame)
```

The target point is simply the car's position plus the pixel displacement:
```cpp
px = frameWidth  / 2.0f + vx * L * frameWidth;
py = frameHeight         + vy * L * frameHeight;
```

Which expands to:
```
px = 39 + step_x
py = 51 + step_y
```

### Concrete example

Line going slightly to the right:
```
vx =  0.30,  vy = -0.95   (mostly forward, slightly right)
L  =  0.75   (medium lookahead)

step_x =  0.30 × 0.75 × 78 =  17.6 pixels right
step_y = -0.95 × 0.75 × 51 = -36.4 pixels up

px = 39 + 17.6 = 56.6
py = 51 - 36.4 = 14.6
```

The target point is at (56.6, 14.6) — upper right area of the frame,
which makes sense for a slight right curve ahead.

Visual representation:
```
(0,0)─────────────────────(78,0)
  │                          │
  │       * (56.6, 14.6)     │  ← target point
  │      ↗                   │
  │     ↗  L steps            │
  │    ↗   along vector       │
  │   ↗                      │
(0,51)────────●──────────(78,51)
             (39,51) ← car position
```

---

## Step 6 — Why only px is used for steering

After computing (px, py), only **px** is passed to the PID controller:

```cpp
float error = px - frameWidth / 2.0f;
//          = 56.6 - 39
//          = 17.6
```

The error is the **horizontal deviation** of the target point from the
frame center:

```
error > 0  →  target is to the right  →  steer right
error < 0  →  target is to the left   →  steer left
error = 0  →  target is centered      →  go straight
```

`py` is not used because the steering only needs to know *left or right*,
not *how far ahead* the target point is. The lookahead distance already
did its job by influencing where the target point ended up horizontally.

---

## Summary of the full pipeline

```
(vx, vy)              unit vector, direction of the line
    │
    ▼
curvature = fabsf(vx) how curved is the road (0=straight, 1=sharp)
    │
    ▼
L = L_MAX - curvature × (L_MAX - L_MIN)   adaptive lookahead fraction
    │
    ▼
step_x = vx × L × frameWidth    pixel displacement from car position
step_y = vy × L × frameHeight
    │
    ▼
px = frameWidth/2  + step_x     target point in frame coordinates
py = frameHeight   + step_y
    │
    ▼
error = px - frameWidth/2       horizontal deviation → fed into PID
```

---

## Constants reference

| Constant | Value | Meaning |
|----------|-------|---------|
| `L_MIN`  | 0.4   | Minimum lookahead (sharp turns) |
| `L_MAX`  | 0.9   | Maximum lookahead (straights)   |
| `frameWidth`  | 78 px | Pixy2 line tracking frame width  |
| `frameHeight` | 51 px | Pixy2 line tracking frame height |
