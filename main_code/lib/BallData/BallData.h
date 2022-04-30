#ifndef BALLDATA_H
#define BALLDATA_H

#include <Common.h>
#include <Point.h>

typedef struct BallData {
    int16_t x;
    int16_t y;
    float angle;
    float dist;
    bool visible;
    bool captured;
    BallData() {
        x = 0;
        y = 0;
        angle = 0;
        dist = 0;
        visible = false;
        captured = false;
    }
    BallData(float Angle, float Dist, bool Visible){
        angle = Angle;
        dist = Dist;
        visible = Visible;
    }
    
} BallData;


#endif