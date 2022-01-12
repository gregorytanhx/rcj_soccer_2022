#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <math.h>

#define pi 3.1415926536
#define e 2.7182818284

float deg2rad(float angle);
float rad2deg(float angle);
float angleDiff(float angle1, float angle2);

double norm(float val, int max, int min);

typedef union floatData {
  float value;
  uint8_t b[4];
} floatData;

typedef union int16Data {
  int16_t value;
  uint8_t b[2];
} int16Data;


#endif