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


};

#endif