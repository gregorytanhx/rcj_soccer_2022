#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Config.h>
#include <Common.h> 
#include <Pins.h> 
#include <TOF.h>

TOF_Array tofArray(Wire);

void setup() {
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