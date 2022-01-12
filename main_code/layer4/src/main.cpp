#include <Arduino.h>
#include <SoftwareSerial.h>

uint8_t i = 0;

SoftwareSerial mySerial(PA10, PA9);
void setup() {
  //pinMode(PA4, OUTPUT);
  mySerial.begin(9600);
}

void loop() {
  //digitalWrite(PA4, LOW);
  mySerial.write(i);
  //Serial1.println("fuck");
  i++;
}