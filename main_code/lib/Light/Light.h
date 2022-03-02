#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include <Common.h>
#include <Math.h>
#include <Timer.h>
#include <Pins.h>

// struct for line data to be sent over serial
typedef struct LineData {
  floatData lineAngle;
  floatData chordLength;
  bool onLine;
} LineData;

// threshold values for calibrating light sensors
typedef union LightThresh {
    int16_t vals[32];
    u_int8_t b[64];
} LightThresh;

// class to control light sensors
class Light {
  public:
    Light();
    void calibrate();
    void init();
    void read();
    void getLineData(LineData& data);
    float lineTrack(float target);
    float getClosestAngle(float angle);

    float lineAngle = 0;
    float lastLineAngle = 0;
    float chordLength = 0;
    bool onLine = false;
    int lineDetected[32];
    int outSensors = 0;

  private:
    int muxChannel[16][4] = {
        {0, 0, 0, 0}, //channel 0
        {1, 0, 0, 0}, //channel 1
        {0, 1, 0, 0}, //channel 2
        {1, 1, 0, 0}, //channel 3
        {0, 0, 1, 0}, //channel 4
        {1, 0, 1, 0}, //channel 5
        {0, 1, 1, 0}, //channel 6
        {1, 1, 1, 0}, //channel 7
        {0, 0, 0, 1}, //channel 8
        {1, 0, 0, 1}, //channel 9
        {0, 1, 0, 1}, //channel 10
        {1, 1, 0, 1}, //channel 11
        {0, 0, 1, 1}, //channel 12
        {1, 0, 1, 1}, //channel 13
        {0, 1, 1, 1}, //channel 14
        {1, 1, 1, 1}  //channel 15
    };
    LightThresh lightThresh;
    LineData lineData;
    int lightVals[32];
    int pinsA[4];
    int pinsB[4];
    int sigA;
    int sigB;
    int readMux(int channel, int controlPin[4], int sig);

    Timer lightTimer = Timer(5000);
    ;
};

#endif