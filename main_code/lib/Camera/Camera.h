#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>
#include <Config.h>
#include <Point.h>
#include <Common.h>

enum Side {
  facingYellow,
  facingBlue, 
  unset
};

typedef union camBuffer {
  int16_t vals[(CAMERA_PACKET_SIZE - 1) / 2];
  uint8_t b[CAMERA_PACKET_SIZE - 1];
} camBuffer;

// class to receive and process data from OpenMV 
class Camera {
  public:
    void begin();
    bool read();
    void process();
    void update();
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

    bool oppVisible = false;
    bool ownVisible = false;
    bool blueVisible = false;
    bool yellowVisible = false;
    bool ballVisible = false;
    bool newData = false;

    Point centreVector;
    Point frontVector;
    Point oppGoalVec;
    Point ownGoalVec;
    Side side = facingYellow; // face yellow goal by default
    camBuffer buffer;
};

#endif