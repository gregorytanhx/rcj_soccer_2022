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

class IMU {
   public:
    IMU();
    void init();
    float readRaw();
    float read();
    Adafruit_BNO055 bno;
    int heading;

   private:
    int offset;
};

#endif