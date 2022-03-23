#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include <Common.h>
#include <EEPROM.h>
#include <Math.h>
#include <MyTimer.h>
#include <Pins.h>

// whether or not to use saved calibration values
#define USE_EEPROM

// microseconds between each channel switch
#define MUX_DELAY 100   

// class to control light sensors
class Light {
   public:
    Light();
    void calibrate();
    void init();
    void read();
    void printLight();
    void printThresh();
    void sendVals();
    void getLineData(LineData& data);
    float lineTrack(float target);
    float getClosestAngle(float angle);
    bool doneReading();

    float lineAngle = 0;
    float lastLineAngle = 0;
    float chordLength = 0;
    bool onLine = false;
    int lineDetected[32];
    int outSensors = 0;

    // int lightMap[32] = {7,  6,  5,  4,  3,  2,  31, 0,  1,  30, 29,
    //                     28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18,
    //                     17, 16, 15, 14, 13, 12, 11, 10, 9,  8};
    int lightMap[32] = {25, 26, 27, 28, 29, 30, 1,  0,  31, 2,  3,
                        4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
                        15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

    int fixedThresh[32] = {751, 714, 745, 738, 724, 736, 742, 738,
                           744, 700, 697, 630, 608, 741, 690, 649,
                           593, 724, 735, 632, 670, 771, 706, 699,
                           633, 577, 775, 772, 743, 740, 768, 686};

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
    int pinsA[4];
    int pinsB[4];
    int sigA;
    int sigB;
    int readMux(int channel, int controlPin[4], int sig);
    int maxVals[32];
    int minVals[32];
    MyTimer lightTimer = MyTimer(10000);
    long readTimer = 0;
    int lightCnt = 0;
};

#endif