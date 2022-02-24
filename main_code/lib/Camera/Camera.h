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

// class to receive and process data from OpenMV 
class Camera {
  public:
    void init();
    void read();
    void process();
    double cmDist(int pixelDist);

    double ballDist;
    double yellowDist;
    double blueDist;

    int ballAngle;
    int ballPixelDist;
    int blueAngle;
    int bluePixelDist;
    int yellowAngle;
    int yellowPixelDist;

    int oppAngle;
    double ownDist;
    int ownAngle;
    double oppDist;

    bool blueVisible = true;
    bool yellowVisible = true;
    bool ballVisible = true;

    Side side = facingYellow; // face yellow goal by default;


};

#endif