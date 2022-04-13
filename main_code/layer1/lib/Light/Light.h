#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include <Common.h>
#include <EEPROM.h>
#include <Math.h>
#include <MyTimer.h>
#include <Pins.h>

// whether or not to use saved calibration values
//#define USE_EEPROM

// microseconds between each channel switch
#define MUX_DELAY 100

// class to control light sensors
class Light {
   public:
    void calibrate();
    void init();
    void read();
    void readRaw();
    void printLight();
    void printThresh();
    void sendVals();
    void getLineData(LineData& data);
    float lineTrack(float target);
    float getClosestAngle(float angle);
    bool doneReading();

    float lineAngle = 0;
    float lineTrackAngle;
    float lastLineAngle = 0;
    float initialLineAngle = 0;
    float chordLength = 0;
    bool onLine = false;
    int lineDetected[32];
    int outSensors = 0;
    float clusterStart;
    float clusterEnd;
    // mapping from light sensor position in MUX to position in circle
    int lightMap[32] = {25, 26, 27, 28, 29, 30, 1,  0,  31, 2,  3,
                        4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
                        15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

    int fixedThresh[32] = {367, 481, 459, 299, 468, 435, 418, 432,
                           416, 399, 308, 367, 367, 459, 407, 468,
                           459, 435, 439, 468, 435, 459, 459, 407,
                           468, 465, 367, 459, 435, 459, 439, 468};

    int muxChannel[16][4] = {
        {0, 0, 0, 0},  // channel 0
        {1, 0, 0, 0},  // channel 1
        {0, 1, 0, 0},  // channel 2
        {1, 1, 0, 0},  // channel 3
        {0, 0, 1, 0},  // channel 4
        {1, 0, 1, 0},  // channel 5
        {0, 1, 1, 0},  // channel 6
        {1, 1, 1, 0},  // channel 7
        {0, 0, 0, 1},  // channel 8
        {1, 0, 0, 1},  // channel 9
        {0, 1, 0, 1},  // channel 10
        {1, 1, 0, 1},  // channel 11
        {0, 0, 1, 1},  // channel 12
        {1, 0, 1, 1},  // channel 13
        {0, 1, 1, 1},  // channel 14
        {1, 1, 1, 1}   // channel 15
    };
    LightBuffer lightThresh;
    LightBuffer lightBuffer;
    LineData lineData;
    int lightVals[32];
    int maxVals[32];
    int minVals[32];
    long readTimer = 0;
    int lightCnt = 0;
};

#endif