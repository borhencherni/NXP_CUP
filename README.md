# ENSI_2024_ROBOCUP_LINEFOLLOWER

A line-following robot built for RoboCup, using a 7-sensor array and a matrix-weighted PID controller with a state machine to handle multiple distinct sections of the competition track (straight lines, forced turns, dashed/inverted-color segments, and zigzag corners).

Thanks to everyone who worked on this project.

![Robot](Bimo.png)

## How it works

The robot reads 7 line sensors and computes an error term to steer via PID, but instead of a single fixed set of sensor weights, it keeps a **5×7 history matrix** of the last 5 sensor readings and multiplies it against one of three weight matrices depending on which part of the track it's on:

- **Neutral matrix** — symmetric weights, no turn bias, used on straight/simple sections
- **Extreme-right matrix** — heavier weights on the right side, used where the track requires prioritizing a right turn
- **Extreme-left matrix** — heavier weights on the left side, for left-turn-priority sections

A `flag` variable acts as a state machine tracking which segment of the track the robot is currently on (there are 9 flags/states in total). Each state transition is triggered by specific sensor patterns (e.g. all sensors seeing black, or the outer sensors both triggering), and each state applies its own combination of weight matrix, PID speed, and sensor filtering.

### Key techniques used

- **Matrix PID** (`RunPIDMatrix`) — the main control method, using the weighted sensor history described above
- **Sensor filtering** (`RightFilterExtreme`, `No9taFilter`) — temporarily "blinds" certain sensors to avoid the robot reacting to small side-markers or noise that would otherwise throw off the PID
- **Line inversion** (`ReadSensorsInverse`) — one section of the track uses a white line instead of black, so sensor logic is inverted for that state
- **Zigzag handling** (`ZigZagFilter`) — detects sharp end-of-line turns and temporarily adjusts speed and weight matrix to complete large turns before resuming normal PID
- **Timed blind runs** (`runForward`) — for sections where the sensors see all-black and PID can't reliably steer, the robot just drives forward for a fixed time/speed instead

The code also includes a basic, unused single-array PID (`PID` using `claculatError`/`penderations`) and an unused `followlinebasic`-style approach, kept in for reference since they were tested and do work, even though the final robot uses the matrix-based version.

## Files

Since this repo has no folders, everything is flat:

- `BIMO_FINAL_CODE.ino` — the full Arduino robot code (sensor reading, PID, state machine, motor control)
- `Bimo.png` — photo of the assembled robot
- `map.png` — diagram of the competition track/map the robot was tuned for
- `README.md` — this file

## Hardware (as wired in the code)

- 2 DC motors, each driven by a pair of PWM pins (forward/backward): `LeftMotor1/2` (pins 3, 6), `RightMotor1/2` (pins 10, 11)
- 7 line sensors on pins 7, 4, A4, 2, A5, 8, 12

## The map

![map](map.png)

The track is split into 9 sections, each requiring different steering behavior (straight PID, right-priority turns, left-priority turns, a dashed/marked segment to ignore, a blind straight run, an inverted white-line segment, and a zigzag finish). The `flag` state machine in the code walks through these sections in order.

## Notes

- The mechanical build (chassis/conception) isn't polished — if you want the CAD/build files, reach out and they'll be shared.
- If anything in the code or the map isn't clear, or a comment doesn't make sense, feel free to get in touch.
