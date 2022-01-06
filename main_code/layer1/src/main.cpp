#include <Arduino.h>


void setup() {
  pinMode(19, OUTPUT);
}

void loop() {
  digitalWrite(19, HIGH);
  delay(1000);
  digitalWrite(19, LOW);
  delay(1000);
}