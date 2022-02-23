#ifndef BALLDATA_H
#define BALLDATA_H

#include <Common.h>
#include <Point.h>
#include <Config.h>

typedef struct BallData {
    int16_t x;
    int16_t y;
    float angle;
    float dist;
    bool visible;
    BallData() {
        x = 0;
        y = 0;
        angle = 0;
        dist = 0;
        visible = false;
    }
    BallData(int16_t X, int16_t Y, bool Visible){
        x = X;
        y = Y;
        visible = Visible;
        Point tmp(x, y);
        angle = tmp.getAngle();
        dist = tmp.getDist();
    }
    
} BallData;


#endif