#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Config.h>
#include <Wire.h>
#include <Common.h> 
#include <Pins.h> 
#include <TOF.h>

TwoWire Wire1(LAYER4_SDA, LAYER4_SCL);
TOF_Array tofArray(Wire1);

void setup() {
    L4CommSerial.begin(STM32_BAUD);
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, HIGH);
    
#ifdef DEBUG
    L4DebugSerial.begin(9600);
#endif
    tofArray.init();
    digitalWrite(STM32_LED, LOW);
}

void loop() {
    tofArray.update();
    tofArray.send();

#ifdef DEBUG
    L4DebugSerial.print("Front: ");
    L4DebugSerial.print(tofArray.buffer.vals[0]);
    L4DebugSerial.print("Right: ");
    L4DebugSerial.print(tofArray.buffer.vals[1]);
    L4DebugSerial.print("Back: ");
    L4DebugSerial.print(tofArray.buffer.vals[2]);
    L4DebugSerial.print("Left: ");
    L4DebugSerial.print(tofArray.buffer.vals[3]);
#endif

}