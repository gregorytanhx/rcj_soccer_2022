#include "TOF.h"

TOF::TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin) {
    sensor = SFEVL53L1X(i2cPort, shutdownPin, interruptPin);
    shutdownPin = shutdownPin;
    interruptPin = interruptPin;
}

void TOF::setLow() {
    pinMode(shutdownPin, OUTPUT);
    pinMode(interruptPin, OUTPUT);
    digitalWrite(shutdownPin, LOW);
}

void TOF::init(int i2cAddress) {

    digitalWrite(shutdownPin, HIGH);
    delay(10);
    sensor.begin();
    sensor.setI2CAddress(i2cAddress);
    // select smallest ROI 
    //sensor.setROI(4, 4, 199);
    sensor.setDistanceModeShort();
    sensor.setTimingBudgetInMs(50);
    sensor.setIntermeasurementPeriod(50);
    
}

int16_t TOF::read() {
    sensor.startRanging();  // Write configuration bytes to initiate measurement
    
    while (!sensor.checkForDataReady()) delay(1);
    digitalWrite(STM32_LED, HIGH);
    digitalWrite(PB1, HIGH);
    int16_t distance = sensor.getDistance();  // Get the result of the
                                              // measurement from the sensor
   
    sensor.clearInterrupt();
    sensor.stopRanging();

    return distance;
}

TOF_Array::TOF_Array(TwoWire &i2cPort) {
    FrontTOF = TOF(i2cPort, SHUT_1, INT_1);
    LeftTOF = TOF(i2cPort, SHUT_2, INT_2);
    BackTOF = TOF(i2cPort, SHUT_3, INT_3);
    RightTOF = TOF(i2cPort, SHUT_4, INT_4);
}

void TOF_Array::init() {
    FrontTOF.setLow();
    BackTOF.setLow();
    LeftTOF.setLow();
    RightTOF.setLow();
   
    FrontTOF.init(0x30);
    BackTOF.init(0x32);
    LeftTOF.init(0x34);
    RightTOF.init(0x36);
    
}

void TOF_Array::update() {
    buffer.vals[0] = FrontTOF.read();
    buffer.vals[1] = RightTOF.read();
    // buffer.vals[2] = BackTOF.read();
    // buffer.vals[3] = LeftTOF.read();
}

void TOF_Array::send() {
    L4CommSerial.write(LAYER4_SYNC_BYTE);
    L4CommSerial.write(buffer.b, 8);
}

void TOF_Array::print() {

    L4DebugSerial.print("Front: ");
    L4DebugSerial.print(buffer.vals[0]);
    L4DebugSerial.print("Right: ");
    L4DebugSerial.print(buffer.vals[1]);
    L4DebugSerial.print("Back: ");
    L4DebugSerial.print(buffer.vals[2]);
    L4DebugSerial.print("Left: ");
    L4DebugSerial.print(buffer.vals[3]);
    L4DebugSerial.println();

}