#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <Config.h>
#include <math.h>

#define pi 3.1415926536

float deg2rad(float angle);
float rad2deg(float angle);
float angleDiff(float angle1, float angle2);
float distance(float x, float y);
float norm(float val, int max, int min);
int sign(int x);
float nonReflex(float angle);
bool angleIsInside(float angleBoundCounterClockwise,
                   float angleBoundClockwise,
                   float angleCheck);
float polarAngle(float y, float x);

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
    floatData initialLineAngle;
    floatData chordLength;
    bool onLine;
} LineData;

// struct for movement data to be sent from teensy to layer1
typedef struct MoveData {
    floatData speed;
    floatData angle;
    floatData rotation;

    MoveData(int moveSpeed, int moveAngle, int moveRotation) {
        speed.val = moveSpeed;
        angle.val = moveAngle;
        rotation.val = moveRotation;
    }
} MoveData;

// struct for TOF values, sent from layer4 to teensy
typedef union TOFBuffer {
    uint16_t vals[4] = {0, 0, 0, 0};
    uint8_t b[8];
} TOFBuffer;

// threshold values for calibrating light sensors
typedef union LightBuffer {
    int16_t vals[32];
    u_int8_t b[64];
} LightBuffer;

#endif