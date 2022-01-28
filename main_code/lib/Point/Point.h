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
    } 

    Point(float ang, float dist) {
        angle = ang;
        distance = dist;
        x = (int)(distance * sin(deg2rad(angle)));
        y = (int)(distance * cos(deg2rad(angle)));
    }

    float getAngle() { 
        // calculate polar angle as bearing from north
        return fmod(90 - rad2deg(atan2(y, x)), 360); 
    }

    float getDistance() {
        // calculate polar distance
        return sqrt(x * x + y * y);
    }

    Point &operator+=(const Point &rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    Point &operator-=(const Point &rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }

    Point operator+(const Point &rhs) { 
        // add two points
        return Point(x + rhs.x, y + rhs.y); 
    }

    Point operator-(const Point &rhs) { 
        // subtract two points
        return Point(x - rhs.x, y - rhs.y); 
    }

    bool operator!=(const Point &p) { return x != p.x || y != p.y; }
};

#endif