#ifndef POINT_H
#define POINT_H

#include <Arduino.h>
#include <Common.h>

class Point {
  public:
    float x;
    float y;
    Point(float posX, float posY) {
      x = posX;
      y = posY;
    }
    float getAngle() {
      return fmod(90 - rad2deg(atan2(y, x)), 360);
    };
    float getMagnitude() {
      return sqrt(x*x + y*y);
    };
}

#endif