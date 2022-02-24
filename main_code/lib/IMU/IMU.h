#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Wire.h>
#include <math.h>
#include <utility/imumaths.h>

// class for BNO055 IMU
class IMU {
   public:
    IMU(TwoWire *theWire);
    void init();
    float readRaw();
    float read();
    Adafruit_BNO055 bno;
    float heading;

   private:
    int offset;
};

#endif