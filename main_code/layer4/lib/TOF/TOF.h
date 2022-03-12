#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <Common.h>
#include <ComponentObject.h>
#include <Config.h>
#include <Pins.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

// wrapper for VL53L1X TOF sensors
class TOF {
   public:
    TOF() {};
    TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin);
    void init(int i2cAddress);
    void setLow();
    SFEVL53L1X sensor;
    int16_t read();

   private:
    int shutdownPin;
    int interruptPin;
};

// class for reading from all TOFs
class TOF_Array {
   public:
    TOF_Array(TwoWire &i2cPort);
    TOF FrontTOF;
    TOF BackTOF;
    TOF LeftTOF;
    TOF RightTOF;
 
    void init();
    void update();
    void send();
    TOFBuffer buffer;
};


#endif