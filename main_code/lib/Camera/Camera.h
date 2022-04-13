#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <Common.h>
#include <Point.h>
#include <math.h>
#include <movingAvg.h>

#define CAMERA_PACKET_SIZE 17
#define CAMERA_SYNC_BYTE 42
#define CAMERA_BAUD 1000000

enum Side { facingYellow, facingBlue, unset };

typedef union camBuffer {
    uint16_t vals[(CAMERA_PACKET_SIZE - 1) / 2];
    uint8_t b[CAMERA_PACKET_SIZE - 1];
} camBuffer;

// class to receive and process data from OpenMV
class Camera {
   public:
    void begin();
    bool read();
    void process();
    void update();
    void printData(int timeOut = 0);
    float getOrientation();

    float ballAngle, ballDist, predBallAngle, predBallDist;
    float blueAngle, blueDist;
    float yellowAngle, yellowDist;

    float oppGoalAngle, ownGoalDist;
    float ownGoalAngle, oppGoalDist;

    bool oppGoalVisible = false;
    bool ownGoalVisible = false;
    bool predBall = false;
    bool blueVisible = false;
    bool yellowVisible = false;
    bool ballVisible = false;
    bool newData = false;

    long lastPrintTime = 0;
    long lastReadTime = 0;

    Point centreVector;
    Point frontVector;
    Point oppGoalVec;
    Point ownGoalVec;
    Side side = facingYellow;  // face yellow goal by default
    camBuffer buffer;
};

#endif