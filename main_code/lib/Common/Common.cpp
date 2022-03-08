#include "Common.h"

float rad2deg(float angle) { return angle * 180 / pi; }

float deg2rad(float angle) { return angle * pi / 180; }

float angleDiff(float angle1, float angle2) {
    float diff = fmod(angle1 - angle2, 360);
    return min(abs(diff), 360 - abs(diff));
}

bool angleIsInside(float angleBoundCounterClockwise,
                   float angleBoundClockwise, float angleCheck) {
    if (angleBoundCounterClockwise < angleBoundClockwise) {
        return (angleBoundCounterClockwise < angleCheck &&
                angleCheck < angleBoundClockwise);
    } else {
        return (angleBoundCounterClockwise < angleCheck ||
                angleCheck < angleBoundClockwise);
    }
}

// normalise to 0 to 1
float norm(float val, int max, int min) { return (val - min) / (max - min); }

float nonReflex(float angle) {
    if (angle > 180) return angle - 360;
    return angle;
}

float distance(float x, float y) { return sqrt(x * x + y * y); }

int sign(int x) { return (x >= 0) ? 1 : -1; }