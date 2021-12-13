#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <math.h>

#define pi 3.1415926536

float deg2rad(float angle);
float rad2deg(float angle);
float angleDiff(float angle1, float angle2);

double norm(float val, int max, int min);

typedef union floatData {
  float f;
  uint8_t b[4];
} floatData;
