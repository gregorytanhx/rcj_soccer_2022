#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Point.h>
#include <math.h>

enum Side { facingYellow, facingBlue, unset };

typedef union camBuffer {
    float vals[(CAMERA_PACKET_SIZE - 1) / 4];
    uint8_t b[CAMERA_PACKET_SIZE - 1];
} camBuffer;

// class to receive and process data from OpenMV
class Camera {
   public:
    void begin();
    bool read();
    void process();
    void update();
    void printData();
    float cmDist(float pixelDist);

    float ballAngle;
    float ballDist;
    float blueAngle;
    float blueDist;
    float yellowAngle;
    float yellowDist;

    float oppGoalAngle;
    float ownGoalDist;
    float ownGoalAngle;
    float oppGoalDist;

    bool oppGoalVisible = false;
    bool ownGoalVisible = false;
    bool blueVisible = false;
    bool yellowVisible = false;
    bool ballVisible = false;
    bool newData = false;

    Point centreVector;
    Point frontVector;
    Point oppGoalVec;
    Point ownGoalVec;
    Side side = unset;  // face yellow goal by default
    camBuffer buffer;
};

#endif