#include <Arduino.h>
#include <SoftwareSerial.h>

uint8_t i = 0;
TOF_Array tofArray;

SoftwareSerial mySerial(PA10, PA9);
void setup() {
    pinMode(PA4, OUTPUT);
    mySerial.begin(9600);
    tofArray.init();
}

void loop() {
    digitalWrite(PA4, LOW);
    tofArray.update();
    tofArray.send();
}