#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055_t3.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <i2c_t3.h>
#include <math.h>
#include <utility/imumaths.h>

class IMU {
   public:
    IMU();
    void init();
    void read();
    Adafruit_BNO055 bno;
    int heading;

   private:
    int offset;
};

#endif