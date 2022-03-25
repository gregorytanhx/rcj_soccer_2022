#ifndef BBOX_H
#define BBOX_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Point.h>
#include <movingAvg.h>

#define DATAPOINTS 5
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
    void begin();
    void update(TOFBuffer tof, LineData lineData, float heading);
    void print();
    void printTOF();
    movingAvg tofAvg[4] = {movingAvg(DATAPOINTS), movingAvg(DATAPOINTS),
                           movingAvg(DATAPOINTS), movingAvg(DATAPOINTS)}
};

#endif