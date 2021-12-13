#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>
#include <Config.h>
#include <Common.h>

#define CAMERA_PACKET_SIZE 13
#define CAMERA_SYNC_BYTE 0x80
#define CAMERA_BAUD 2000000

enum Side {
  facingYellow,
  facingBlue, 
  unset
};

typedef union cameraBuffer {
  int16_t vals[(CAMERA_PACKET_SIZE - 1) / 2];
  uint8_t b[CAMERA_PACKET_SIZE - 1];
} cameraBuffer;

class Camera {
  public:
    void init();
    void read();
    void process();
    double cmDist(int pixelDist);

    double ballDist;
    double yellowDist;
    double blueDist;

    int16Data blueAngle;
    int16Data bluePixelDist;
    int16Data yellowAngle;
    int16Data yellowPixelDist;
    int16Data ballAngle;
    int16Data ballPixelDist;

    int oppAngle;
    int ownDist;
    int ownAngle;
    int ownDist;

    bool blueVisible = true;
    bool yellowVisible = true;
    bool ballVisible = true;

    Side side = facingYellow; // face yellow goal by default;

    cameraBuffer buffer;

}