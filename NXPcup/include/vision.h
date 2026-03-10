#ifndef VISION_H
#define VISION_H

#include <Pixy2.h>

// Initialize Pixy camera
void Vision_Init();

// Returns line error relative to image center
int Vision_GetLineError();

// Debug function to print detected features
void Vision_PrintFeatures();


#endif