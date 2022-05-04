#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Common.h>
#include <Wire.h>
#include <math.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#define USE_LAYER4_IMU

#ifdef USE_LAYER4_IMU
    #include <Adafruit_BNO055.h>
    #define Serial L4DebugSerial
#else
    #include <Adafruit_BNO055_t4.h>
#endif




// class for BNO055 IMU
class IMU {
   public:
    IMU(TwoWire *theWire);
    void begin();
    void printData();
    void calibrate();
    void sendCalib();
    void displaySensorDetails();
    void displaySensorOffsets(const adafruit_bno055_offsets_t& calibData);
    void printCalib();
    void loadCalib();
    double readQuat();
    float readEuler();
    float read();
    Adafruit_BNO055 bno;
    float heading;
    float eulerOffset = 0;
    float quatOffset = 0;
    float angleOffset = 0;
    int eeAddress = IMU_CALIB_ADDR;
    sensors_event_t event;
    adafruit_bno055_offsets_t calibrationData;
};

#endif