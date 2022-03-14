#include <Arduino.h>
#include <Common.h>
#include <ComponentObject.h>
#include <Config.h>
#include <Pins.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

TwoWire Wire1(PB11, PB10);

int shutPins[4] = {SHUT_1, SHUT_2, SHUT_3, SHUT_4};
int intPins[4] = {INT_1, INT_2, INT_3, INT_4};

TOFBuffer buffer;

// front, left, back, right
SFEVL53L1X sensors[] = {
    SFEVL53L1X(Wire1, SHUT_1, INT_1), SFEVL53L1X(Wire1, SHUT_2, INT_2),
    SFEVL53L1X(Wire1, SHUT_3, INT_3), SFEVL53L1X(Wire1, SHUT_4, INT_4)};

void init_sensors() {
    // Shut down all sensors
    for (int i = 0; i < 4; i++) {
        pinMode(shutPins[i], OUTPUT);
        pinMode(intPins[i], OUTPUT);
        digitalWrite(shutPins[i], LOW);
    }

    // set i2c addresses one by one
    for (int i = 0; i < 4; i++) {
        digitalWrite(shutPins[i], HIGH);
        sensors[i].init();
        sensors[i].setI2CAddress(0x30 + i * 2);
        sensors[i].setDistanceModeLong();
        sensors[i].setTimingBudgetInMs(TIME_BUDGET);
        sensors[i].setIntermeasurementPeriod(IMP);
        delay(10);
    }
}

void read_sensors() {
    for (int i = 0; i < 4; i++) {
        sensors[i].startRanging();  // Write configuration bytes to initiate
                                    // measurement
        while (!sensors[i].checkForDataReady()) {
            delay(1);
        }
        buffer.vals[i] =
            sensors[i].getDistance();  // Get the result of the
                                       // measurement from the sensor
        sensors[i].clearInterrupt();
        sensors[i].stopRanging();
    }
}

void sendVals() {
    L4CommSerial.write(LAYER4_SYNC_BYTE);
    L4CommSerial.write(buffer.b, sizeof(buffer.b));
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
    init_sensors();
    digitalWrite(STM32_LED, HIGH);
}

void loop() {
    read_sensors();
#ifdef DEBUG
    for (int i = 0; i < 4; i++) {
        L4DebugSerial.print(distances[i]);
        L4DebugSerial.print(" ");
    }
    L4DebugSerial.println();
#endif
    sendVals();
}