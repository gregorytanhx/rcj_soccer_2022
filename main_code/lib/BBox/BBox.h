#ifndef BBOX_H
#define BBOX_H

#include <Config.h>
#include <Common.h>
#include <Arduino.h>
#include <Point.h>

// class for bounding box for robot's position
class BBox {
   public:
    int width;
    int height;
    int x;
    int y;
    int Xstart;
    int Xend;
    int Ystart;
    int Yend;
    float Xconfidence;
    float Yconfidence;
    void update(TOFBuffer tof, LineData lineData, float heading);
};

#endif