#include <Arduino.h>
#include <Common.h>
#include <Pins.h>
#include <IMU.h>
#include <Wire.h>

TwoWire Wire1(PB11, PB10);

IMU cmp(&Wire1);

#define Serial L4DebugSerial

CmpVal cmpVal;

void sendData() {
    cmpVal.val = (int) (cmp.readQuat() * 100);
    Serial2.write(IMU_SYNC_BYTE);
    Serial2.write(cmpVal.b, 2);
}


void setup() {    
    Serial2.begin(STM32_BAUD);
#ifdef DEBUG
    Serial.begin(9600);
#endif
    Serial.println("test");
    delay(3000);
    cmp.begin();
    // pinMode(STM32_LED, OUTPUT);
    // digitalWrite(STM32_LED, HIGH);
}
long lastPrintTime = 0;
void loop() {    
    sendData();
    // Serial.println(cmp.readQuat());
  
}



