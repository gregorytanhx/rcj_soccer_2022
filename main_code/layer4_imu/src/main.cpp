#include <Arduino.h>
#include <Common.h>
#include <Pins.h>
#include <Config.h>
#include <IMU.h>
#include <Wire.h>

TwoWire Wire1(PB11, PB10);

IMU cmp(&Wire1);

#define Serial L4DebugSerial


CmpVal cmpVal;

void sendData() {
    cmpVal.val = cmp.readQuat();
    L4CommSerial.write(IMU_SYNC_BYTE);
    L4CommSerial.write(cmpVal.b, 4);
}

void setup() {    
    L4CommSerial.begin(STM32_BAUD);
#ifdef DEBUG
    Serial.begin(9600);
#endif
    Serial.println("test");
    cmp.begin();
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, HIGH);
}
long lastPrintTime = 0;
void loop() {
    if (millis() - lastPrintTime > 250) {
        lastPrintTime = millis();
        cmp.printCalib();
        Serial.print("Euler: ");
        Serial.println(cmp.readEuler());
        Serial.print("Quaternion: ");
        Serial.println(cmp.readQuat());
    }
    sendData();
    
}


