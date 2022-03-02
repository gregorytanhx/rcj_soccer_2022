#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <Config.h>
#include <math.h>

#define pi 3.1415926536
#define e 2.7182818284

float deg2rad(float angle);
float rad2deg(float angle);
float angleDiff(float angle1, float angle2);
double distance(double x, double y);
double norm(float val, int max, int min);
int sign(int x);
double nonReflex(double angle);

typedef union floatData {
    float val;
    uint8_t b[4];
} floatData;

typedef union int16Data {
    int16_t val;
    uint8_t b[2];
} int16Data;


typedef union motorBuffer {
    uint16_t vals[3];
    uint8_t b[sizeof(vals)];
} motorBuffer;



#endif