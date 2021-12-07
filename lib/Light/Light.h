#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include <Common.h>
#include <Math.h>
#include <Pins.h>

class Light{
  public:
    
    Light();
    void calibrate();
    int readMux(int channel, int controlPin[4], int sig);
    bool readLight();
    float getLineData();

    
    float lineAngle;
    float chordLength;
    bool onLine = false;
    int lightVals[32];
    int pinsA[4];
    int pinsB[4];
    int sigA;
    int sigB;

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
    int lightThresh[32] = {844, 875, 886, 882, 882, 896, 899, 881, 889, 892, 880, 881, 900, 862, 888, 725,
                           900, 888, 890, 877, 892, 862, 894, 897, 884, 886, 892, 904, 887, 893, 869, 606};
}