#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>
#include <Config.h>
#include <Common.h>


enum Side {
  facingYellow,
  facingBlue, 
  unset
};

typedef union camBuffer {
  int16_t vals[6];
  uint8_t b[12];
} camBuffer;

// class to receive and process data from OpenMV 
class Camera {
  public:
    void init();
    void read();
    void process();
    float cmDist(float pixelDist);

    float ballDist;
    float yellowDist;
    float blueDist;

    float ballAngle;
    int ballPixelDist;
    float blueAngle;
    int bluePixelDist;
    float yellowAngle;
    int yellowPixelDist;

    float oppAngle;
    float ownDist;
    float ownAngle;
    float oppDist;

    bool oppVisible = true;
    bool ownVisible = true;
    bool blueVisible = false;
    bool yellowVisible = false;
    bool ballVisible = false;

    Side side = facingYellow; // face yellow goal by default;
    camBuffer buffer;

};

#endif