#include "Common.h"

double rad2deg(double angle) {
  return angle * 180/pi;
}

double deg2rad(double angle) {
  return angle * pi/180;
}

float angleDiff(float angle1, float angle2) {
  return min(abs(angle1 - angle2), 360 - abs(angle1 - angle2));
}

double norm(float val, int max, int min) {
  return (val - min)/(max - min)
}

double nonReflex(double angle) {
  if (angle > 180) return angle - 360;
  return angle;
}