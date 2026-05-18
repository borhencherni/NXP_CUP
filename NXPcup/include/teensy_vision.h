#ifndef TEENSY_VISION_H
#define TEENSY_VISION_H

void Vision_Init();

bool isNinetyDegree(int i);
float computeMedian(float *arr, int size);
void normalizeVectors();
float vectorWeight(int i);
bool validVector(int i);
void fusedVector(float &vx, float &vy);
void normalizeFusedVector(float &vx, float &vy);
float adaptiveLookahead(float vx);
void lookaheadPoint(float vx, float vy, float &px, float &py);
float computeSteering(float px, float dt);
bool isSharpTurn(float steering);
bool isIntersectionPixy(int i, int j);
void detectLaneSides();
bool isCenterDash(int i);
bool detectDashPair();
bool updateDashDetection();
bool isWideHorizontal(int i);
bool detectFinishLine();

#endif