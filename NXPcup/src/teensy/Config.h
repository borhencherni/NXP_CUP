#pragma once

//define pins
#define IN1 23
#define IN2 22
#define IN3 21
#define IN4 20
#define TXESP 15
#define RXESP 14
#define Servo_PIN 19
#define Channel_A_LEFT 2
#define Channel_B_LEFT 3
#define Channel_A_RIGHT 4
#define Channel_B_RIGHT 5

//pixy2 Aspect ratio correction
// Aspect ratio correction: Pixy2 pixels are not square
// frameWidth=78, frameHeight=51 → real ratio ≈ 1.53
//frameWidth  = 78 pixels   →  covers the full horizontal FOV
//frameHeight = 51 pixels   →  covers the full vertical FOV
//ratio = frameWidth / frameHeight = 78 / 51 ≈ 1.53
//This means 1 pixel of height = 1.53 pixels of width in real-world distance.
#define SCALE_Y(y)        ((y) * 1.53f)


// Vector filtering
#define MIN_VECTOR_LEN    15.0f    // ignore very short noise vectors
#define MIN_VECTOR_ANGLE  5.0f     // ignore near-horizontal vectors (deg)


// Lookahead 
#define L_MIN             0.4f     // short lookahead in tight turns (reactive)
#define L_MAX             0.9f     // long lookahead on straights (smooth)

//STEERING PID GAINS 
#define KP                0.80f
#define KI                0.0f    
#define KD                0.0f 
#define I_MAX             15.0f   // Integral windup clamp

// servo variables
#define SERVO_CENTER      90
#define SERVO_MIN         70       
#define SERVO_MAX         110  

// Low-pass filter alpha 
#define LPF_ALPHA 0.7f

//SPEED PID GAINS 
#define KP_S                0.80f
#define KI_S                0.0f    
#define KD_S                0.0f 
#define I_MAX_S             15.0f   // Integral windup clamp

//nombre de ticks pour une rotation complète
#define nb_t_p_rot 22

#define SPEED_PERIOD_MS 100  // 100ms = 10Hz control loop


#define STEERING_PIXEL_SCALE  40.0f //imaginary triangle constant in steering PID




