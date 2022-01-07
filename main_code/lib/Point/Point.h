#ifndef POINT_H
#define POINT_H

#include <Arduino.h>
#include <Common.h>

class Point {
  // include both polar and cartesian coordinates
  public:
    int x;
    int y;
    float angle;
    float distance;

    Point(int posX, int posY) {
      x = posX;
      y = posY;
      angle = fmod(90 - rad2deg(atan2(y, x)), 360);
      distance = sqrt(x * x + y * y);
    } 
    Point (float angleF, float distanceF) {
      angle = angleF;
      distance = distanceF;
      x = (int) (distance * sin(deg2rad(angle)));
      y = (int) (distance * cos(deg2rad(angle)));
    }
};

#endif