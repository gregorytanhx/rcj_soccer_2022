#include "TOF.h"

TOF::TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin) {
    sensor = SFEVL53L1X(i2cPort, shutdownPin, interruptPin);
    shutdownPin = shutdownPin;
    interruptPin = interruptPin;
    pinMode(shutdownPin, OUTPUT);
    pinMode(interruptPin, OUTPUT);
    digitalWrite(shutdownPin, LOW);
}

void TOF::init(int i2cAddress) {
    digitalWrite(shutdownPin, HIGH);
    sensor.init();
    sensor.setI2CAddress(i2cAddress);
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
    Serial1.begin(STM32_BAUD);
    FrontTOF.init();
    delay(10);
    BackTOF.init();
    delay(10);
    LeftTOF.init();
    delay(10);
    RightTOF.init();
    delay(10);
}

void TOF_Array::update() {
    buffer.vals[0] = FrontTOF.read();
    buffer.vals[1] = RightTOF.read();
    buffer.vals[2] = BackTOF.read();
    buffer.vals[3] = LeftTOF.read();
}

void TOF_Array::send() {
    Serial1.write(LAYER4_SYNC_BYTE);
    Serial1.write(buffer.b, 8);
}

void BBox::update(TOFBuffer tof) {
    frontTOF = tof.vals[0];
    rightTOF = tof.vals[1];
    backTOF = tof.vals[2];
    leftTOF = tof.vals[3];

    Xstart = leftTOF;
    Xend = FIELD_WIDTH - rightTOF;
    Ystart = frontTOF;
    Yend = FIELD_HEIGHT - backTOF;

    width = Xend - Xstart;
    height = Yend - Ystart;

    // recenter to obtain coordinates
    Xstart -= FIELD_WIDTH / 2;
    Xend -= FIELD_WIDTH / 2;
    Ystart = FIELD_HEIGHT / 2 - Ystart;
    Yend = FIELD_HEIGHT / 2 - Yend;

    x = (Xstart - Xend) / 2;
    y = (Ystart - Yend) / 2;

    // take area of robot over area of bbox as confidence score
    Xconfidence = 180 / width;
    Yconfidence = 180 / height;
    
}