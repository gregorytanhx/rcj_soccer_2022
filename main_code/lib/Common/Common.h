#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <Config.h>
#include <math.h>

#define pi 3.1415926536

double deg2rad(double angle);
double rad2deg(double angle);
float angleDiff(float angle1, float angle2);
double distance(double x, double y);
double norm(float val, int max, int min);
int sign(int x);
double nonReflex(double angle);
bool angleIsInside(double angleBoundCounterClockwise,
                   double angleBoundClockwise,
                   double angleCheck);

typedef union floatData {
    float val;
    uint8_t b[4];
} floatData;

typedef union int16Data {
    int16_t val;
    uint8_t b[2];
} int16Data;


typedef union motorBuffer {
    float vals[3];
    uint8_t b[sizeof(vals)];
} motorBuffer;

// struct for line data to be sent over serial
typedef struct LineData {
    floatData lineAngle;
    floatData chordLength;
    bool onLine;
} LineData;

#endif