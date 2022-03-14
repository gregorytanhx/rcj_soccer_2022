#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055_t4.h>
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
    void printAllData();
    void printCalib();
    void printEvent(sensors_event_t* event);
    float readRaw();
    float read();
    Adafruit_BNO055 bno;
    float heading;
    float offset = 0;
};

#endif