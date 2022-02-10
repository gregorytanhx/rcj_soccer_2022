#include "IMU.h"

IMU::IMU(TwoWire *theWire) {
    // bno = Adafruit_BNO055(WIRE2_BUS, -1, 0x29, I2C_MASTER, I2C_PINS_3_4,
    //                       I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);
    bno = Adafruit_BNO055(55, 0x29, theWire);
}

void IMU::init() {
    if (!bno.begin()){
        while (1);
    } 
    delay(1000);
    bno.setExtCrystalUse(true);
    read();
    offset = heading;
}

float IMU::readRaw() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    heading = euler.x();
    heading -= offset;
    return heading;
}

float IMU::read() {
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;
    heading -= offset;
    return heading;
}