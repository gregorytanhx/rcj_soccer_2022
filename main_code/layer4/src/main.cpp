#include <Arduino.h>
#include <Common.h>
#include <ComponentObject.h>
#include <Pins.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

TwoWire Wire1(PB11, PB10);

int shutPins[4] = {SHUT_1, SHUT_2, SHUT_3, SHUT_4};
int intPins[4] = {INT_1, INT_2, INT_3, INT_4};
bool doneMeasuring[4];
int tofCnt = 4;

TOFBuffer buffer;

// front, left, back, right
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
        sensors[i].setDistanceModeLong();
        sensors[i].setROI(4, 4, 199);
        sensors[i].setTimingBudgetInMs(100);
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

  
    if (tofCnt == 4){
        // for (int i = 0; i < 4; i++) {
        //     L4DebugSerial.print(buffer.vals[i]);
        //     L4DebugSerial.print(" ");
        // }
        // L4DebugSerial.println();
        sendVals();
    }
}