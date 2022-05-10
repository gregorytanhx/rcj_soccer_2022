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
bool newData;

TOFBuffer buffer;

// front, right, back, left
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
        sensors[i].setDistanceModeShort();
        sensors[i].setROI(4, 4, 199);
        // Intermeasurement Period must be greater than or equal to time budget

        // set shorter time budget for horizontal tofs
        int timeBudget = (i == 1 || i == 3) ? 20 : 33;
        
        sensors[i].setTimingBudgetInMs(timeBudget);
        sensors[i].setIntermeasurementPeriod(timeBudget);
        delay(10);
    }
    for (int i = 0; i < 4; i++) {
        sensors[i].startRanging();
    }
}

void read_sensors() {
    newData = false;
    for (int i = 0; i < 4; i++) {
        // get measurement once ranging is done
        if (sensors[i].checkForDataReady()) {
            buffer.vals[i] = sensors[i].getDistance();
            sensors[i].clearInterrupt();
            sensors[i].stopRanging();
            sensors[i].startRanging();
            newData = true;
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
    // digitalWrite(STM32_LED, HIGH);
}
long tmp;
void loop() {
    // i2cScanner(&Wire1);
    //  L4DebugSerial.println(tofCnt);
    
    // if (newData) {
    //     for (int i = 0; i < 4; i++) {
    //         L4DebugSerial.print(buffer.vals[i]);
    //         L4DebugSerial.print(" ");
    //     }
    // }
        
    L4DebugSerial.println();   
    if (tofCnt == 4) {
        tmp = millis();
    }
    if (millis() - tmp > 3000) digitalWrite(STM32_LED, HIGH);
  
    read_sensors();
    if (newData) sendVals();
}