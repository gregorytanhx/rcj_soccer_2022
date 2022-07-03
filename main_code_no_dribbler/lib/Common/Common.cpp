#include "Common.h"

float rad2deg(float angle) { return angle * 180 / pi; }

float deg2rad(float angle) { return angle * pi / 180; }

float angleDiff(float x, float y) {
    float diff = angleBetween(x, y);
    return fmin(diff, 360  -diff);
}

float truncate(float x, int precision ) {
    int tmp = x * pow(10, precision);
    return tmp / pow(10, precision);
}

float angleAverage(float angle1, float angle2) {
    float avgX = sin(deg2rad(angle1)) + sin(deg2rad(angle2));
    float avgY = cos(deg2rad(angle1)) + cos(deg2rad(angle2));
    return rad2deg(atan2(avgX, avgY));
}

float angleBetween(float x, float y) { return mod((y - x), 360); }

float midAngleBetween(float x, float y) {
    return mod(x + angleBetween(x, y) / 2.0, 360);
}

bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise,
                   float angleCheck) {
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

float mod(float x, float y) {
    x = fmod(x, y);
    return x < 0 ? x + y : x;
}

float distance(float x, float y) { return sqrt(x * x + y * y); }

int sign(int x) { return (x >= 0) ? 1 : -1; }

float polarAngle(float y, float x) {
    // return angle as bearing from north
    return fmod(90 - rad2deg(atan2(y, x)), 360);
}

void i2cScanner(TwoWire &Wire1) {
    byte error, address;  // variable for error and I2C address
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000);  // wait 5
}