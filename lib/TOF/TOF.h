#ifndef TOF_H
#define TOF_H

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Pins.h>
#include <Config.h>

class TOF{
  public:
    TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin)
    SFEVL53L1X sensor;
    void init(int i2cAddress);
    int16_t read();

  private:
    int shutdownPin;
    int interruptPin;
}

class TOF_Array{
  public:
    TOF_Array();
    TOF FrontTOF;
    TOF BackTOF;
    TOF LeftTOF;
    TOF RightTOF;
    void init();
    void update();
    int16_t distances[4] = {0, 0, 0, 0};

}