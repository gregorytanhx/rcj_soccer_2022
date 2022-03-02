#include "Common.h"

double rad2deg(double angle) {
  return angle * 180/pi;
}

double deg2rad(double angle) {
  return angle * pi/180;
}

float angleDiff(float angle1, float angle2) {
  float diff = fmod(angle1 - angle2, 360);
   return min(abs(diff), 360 - abs(diff));
}

bool angleIsInside(double angleBoundCounterClockwise, double angleBoundClockwise, double angleCheck) {
    if (angleBoundCounterClockwise < angleBoundClockwise) {
        return (angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
    } else {
        return (angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
    }
}

// normalise to 0 to 1
double norm(float val, int max, int min) {
    return (val - min) / (max - min);
}

double nonReflex(double angle) {
    if (angle > 180) return angle - 360;
    return angle;
}

double distance(double x, double y) { return sqrt(x * x, y * y); }

int sign(int x) { return (x >= 0) ? 1 : -1; }