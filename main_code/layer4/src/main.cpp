#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Pins.h>
#include <TOF.h>
#include <ComponentObject.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

TwoWire Wire1(PB11, PB10);
TOF_Array tofArray(Wire1);

shutPins = {SHUT_1, SHUT_2, SHUT_3, SHUT_4};
intPins = {INT_1, INT_1}

SFEVL53L1X sensor = {
    SFEVL53L1X(Wire1, SHUT_1, INT_1), SFEVL53L1X(Wire1, SHUT_2, INT_2),
    SFEVL53L1X(Wire1, SHUT_3, INT_3), SFEVL53L1X(Wire1, SHUT_4, INT_4)};

uint16_t distances[4];

void init_sensors() {
    // Shut down all sensors
    pinMode(SHUT_1, OUTPUT);
    pinMode(INT_1, OUTPUT);
    digitalWrite(SHUT_1, LOW);
    pinMode(SHUT_2, OUTPUT);
    pinMode(INT_2, OUTPUT);
    digitalWrite(SHUT_2, LOW);
    pinMode(SHUT_3, OUTPUT);
    pinMode(INT_3, OUTPUT);
    digitalWrite(SHUT_3, LOW);
    pinMode(SHUT_4, OUTPUT);
    pinMode(INT_4, OUTPUT);
    digitalWrite(SHUT_4, LOW);
    // set i2c addresses one by one
    // set addresses with difference of 2 cus source code fucking gay
    digitalWrite(SHUT_1, HIGH);
    distanceSensor1.init();
    distanceSensor1.setI2CAddress(0x30);
    distanceSensor1.setDistanceModeLong();
    distanceSensor1.setTimingBudgetInMs(TIME_BUDGET);
    distanceSensor1.setIntermeasurementPeriod(IMP);
    delay(10);

    digitalWrite(SHUT_2, HIGH);
    distanceSensor2.init();
    distanceSensor2.setI2CAddress(0x32);
    distanceSensor2.setDistanceModeLong();
    distanceSensor2.setTimingBudgetInMs(TIME_BUDGET);
    distanceSensor2.setIntermeasurementPeriod(IMP);
    delay(10);

    digitalWrite(SHUT_3, HIGH);
    distanceSensor3.init();
    distanceSensor3.setI2CAddress(0x34);
    distanceSensor3.setDistanceModeLong();
    distanceSensor3.setTimingBudgetInMs(TIME_BUDGET);
    distanceSensor3.setIntermeasurementPeriod(IMP);
    delay(10);

    digitalWrite(SHUT_4, HIGH);
    distanceSensor4.init();
    distanceSensor4.setI2CAddress(0x36);
    distanceSensor4.setDistanceModeLong();
    distanceSensor4.setTimingBudgetInMs(TIME_BUDGET);
    distanceSensor4.setIntermeasurementPeriod(IMP);
    delay(10);
}

void read_sensors() {
    distanceSensor1
        .startRanging();  // Write configuration bytes to initiate measurement
    while (!distanceSensor1.checkForDataReady()) {
        delay(1);
    }
    distances[0] =
        distanceSensor1.getDistance();  // Get the result of the measurement
                                        // from the sensor
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();

    distanceSensor2
        .startRanging();  // Write configuration bytes to initiate measurement
    while (!distanceSensor2.checkForDataReady()) {
        delay(1);
    }
    distances[1] =
        distanceSensor2.getDistance();  // Get the result of the measurement
                                        // from the sensor
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();

    distanceSensor3
        .startRanging();  // Write configuration bytes to initiate measurement
    while (!distanceSensor3.checkForDataReady()) {
        delay(1);
    }
    distances[2] =
        distanceSensor3.getDistance();  // Get the result of the measurement
                                        // from the sensor
    distanceSensor3.clearInterrupt();
    distanceSensor3.stopRanging();

    distanceSensor4
        .startRanging();  // Write configuration bytes to initiate measurement
    while (!distanceSensor4.checkForDataReady()) {
        delay(1);
    }
    distances[3] =
        distanceSensor4.getDistance();  // Get the result of the measurement
                                        // from the sensor
    distanceSensor4.clearInterrupt();
    distanceSensor4.stopRanging();
}

void i2c_scanner() {
    byte error, address;
    int nDevices;

    L4DebugSerial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();

        if (error == 0) {
            L4DebugSerial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            L4DebugSerial.print(address, HEX);
            L4DebugSerial.println("  !");

            nDevices++;
        } else if (error == 4) {
            L4DebugSerial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            L4DebugSerial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        L4DebugSerial.println("No I2C devices found");
    else
        L4DebugSerial.println("done");
}

void setup() {

    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, LOW);
    L4CommSerial.begin(STM32_BAUD);
#ifdef DEBUG
    L4DebugSerial.begin(9600);
#endif
    Wire1.begin();
    Wire1.setClock(400000);   
    //tofArray.init();
    init_sensors();   
}


void loop() {
    //L4DebugSerial.println("FUCK WPRiopde0riiwe0rjiewrjiwefgihj0fof90pefweperf");
    // i2c_scanner();
    // tofArray.update();
    // digitalWrite(STM32_LED, HIGH);
    // tofArray.print();
    read_sensors();
    for (int i = 0; i < 4; i++) {
        L4DebugSerial.print(distances[i]);
        L4DebugSerial.print(" ");
    }
    L4DebugSerial.println();
}