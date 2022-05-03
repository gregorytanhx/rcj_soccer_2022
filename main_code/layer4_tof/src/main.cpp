#include <Arduino.h>
#include <Common.h>
#include <ComponentObject.h>
#include <Pins.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

#define Serial L4DebugSerial
TwoWire Wire1(PB11, PB10);

int shutPins[4] = {SHUT_1, SHUT_2, SHUT_3, SHUT_4};
int intPins[4] = {INT_1, INT_2, INT_3, INT_4};
bool doneMeasuring[4];
int tofCnt = 4;

TOFBuffer buffer;

// front, right, back, left
SFEVL53L1X sensors[] = { SFEVL53L1X(Wire1, SHUT_1, INT_1),
                         SFEVL53L1X(Wire1, SHUT_2, INT_2),
                         SFEVL53L1X(Wire1, SHUT_3, INT_3), 
                         SFEVL53L1X(Wire1, SHUT_4, INT_4) };

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
        sensors[i].setDistanceModeShort();
        sensors[i].setROI(4, 4, 199);
        // Intermeasurement Period must be greater than or equal to time budget
        sensors[i].setTimingBudgetInMs(33);
        sensors[i].setIntermeasurementPeriod(33);
        delay(10);
    }
}

void read_sensors() {
    // measure again once all sensors have been read
    if (tofCnt == 4) {
        for (int i = 0; i < 4; i++) {
            // Write configuration bytes to initiate
            // measurement
            sensors[i].startRanging();
            tofCnt = 0;
            doneMeasuring[i] = false;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            // get measurement once ranging is done
            if (!doneMeasuring[i] && sensors[i].checkForDataReady()) {
                buffer.vals[i] = sensors[i].getDistance();
                sensors[i].clearInterrupt();
                sensors[i].stopRanging();
                doneMeasuring[i] = true;
                tofCnt++;
            }
        }
    }
}

void sendVals() {
    L4CommSerial.write(LAYER4_SYNC_BYTE);
    L4CommSerial.write(buffer.b, sizeof(buffer.b));
}

void i2cScanner() {
    byte error, address;  // variable for error and I2C address
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000);  // wait 5
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
    //digitalWrite(STM32_LED, HIGH);
   
}

void loop() {
    //i2cScanner();
    // L4DebugSerial.println(tofCnt);
    if (tofCnt == 4) {
        // for (int i = 0; i < 4; i++) {
        //     L4DebugSerial.print(buffer.vals[i]);
        //     L4DebugSerial.print(" ");
        // }
        // L4DebugSerial.println();
        sendVals();
    }
    read_sensors();
}