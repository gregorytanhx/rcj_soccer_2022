#include "IMU.h"

IMU::IMU() {
  bno = Adafruit_BNO055(WIRE2_BUS, -1, 0x29, I2C_MASTER, I2C_PINS_3_4, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);
}

void IMU::init() {
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  read();
  offset = heading;
}

void IMU::readRaw() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  heading = euler.x();
  heading -= offset;
}

void IMU::read() {
  sensors_event_t event;
  bno.getEvent(&event);
  heading = event.orientation.x;
  heading -= offset;
}