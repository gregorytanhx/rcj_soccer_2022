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

    int ballAngle;
    int ballPixelDist;
    int blueAngle;
    int bluePixelDist;
    int yellowAngle;
    int yellowPixelDist;

    int oppAngle;
    float ownDist;
    int ownAngle;
    float oppDist;

    bool blueVisible = true;
    bool yellowVisible = true;
    bool ballVisible = true;

    Side side = facingYellow; // face yellow goal by default;
    camBuffer buffer;

};

#endif