#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Pins.h>
#include <IMU.h>
#include <Wire.h>

TwoWire Wire1(PB11, PB10);

IMU cmp(&Wire1);

#define Serial Serial1

void setup() {    
    L4CommSerial.begin(STM32_BAUD);
#ifdef DEBUG
    Serial.begin(9600);
#endif
    //L4DebugSerial.println("test");
    cmp.init();
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, LOW);
}

void loop() {
    cmp.printCalib();
    Serial.print("Euler: ");
    Serial.println(cmp.readEuler());
    Serial.print("Quaternion: ");
    Serial.println(cmp.readQuat());
    delay(200);
}


