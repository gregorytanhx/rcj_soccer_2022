#include "TOF.h"

TOF::TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin) {
    sensor = SFEVL53L1X(i2cPort, shutdownPin, interruptPin);
    shutdownPin = shutdownPin;
    interruptPin = interruptPin;
    pinMode(shutdownPin, OUTPUT);
    pinMode(interruptPin, OUTPUT);
}

void TOF::setLow() {
    digitalWrite(shutdownPin, LOW);
}

void TOF::init(int i2cAddress) {
    digitalWrite(shutdownPin, HIGH);
    sensor.init();
    sensor.setI2CAddress(i2cAddress);
    // select smallest ROI 
    sensor.setROI(4, 4, 199);
    sensor.setDistanceModeLong();
    sensor.setTimingBudgetInMs(TIME_BUDGET);
    sensor.setIntermeasurementPeriod(IMP);
}

int16_t TOF::read() {
    sensor.startRanging();  // Write configuration bytes to initiate measurement
    while (!sensor.checkForDataReady()) delay(1);
    int16_t distance = sensor.getDistance();  // Get the result of the
                                              // measurement from the sensor
    sensor.clearInterrupt();
    sensor.stopRanging();

    return distance;
}

TOF_Array::TOF_Array(TwoWire &i2cPort) {
    FrontTOF = TOF(i2cPort, SHUT_1, INT_1);
    BackTOF = TOF(i2cPort, SHUT_2, INT_2);
    LeftTOF = TOF(i2cPort, SHUT_3, INT_3);
    RightTOF = TOF(i2cPort, SHUT_4, INT_4);
}

void TOF_Array::init() {
    L4CommSerial.begin(STM32_BAUD);

    FrontTOF.setLow();
    BackTOF.setLow();
    LeftTOF.setLow();
    RightTOF.setLow();

    FrontTOF.init(0x30);
    delay(10);
    BackTOF.init(0x32);
    delay(10);
    LeftTOF.init(0x34);
    delay(10);
    RightTOF.init(0x36);
    delay(10);
}

void TOF_Array::update() {
    buffer.vals[0] = FrontTOF.read();
    buffer.vals[1] = RightTOF.read();
    buffer.vals[2] = BackTOF.read();
    buffer.vals[3] = LeftTOF.read();
}

void TOF_Array::send() {
    L4CommSerial.write(LAYER4_SYNC_BYTE);
    L4CommSerial.write(buffer.b, 8);
}
