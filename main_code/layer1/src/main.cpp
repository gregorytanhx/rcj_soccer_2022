#include <Arduino.h>


void setup() {
  pinMode(PB1, OUTPUT);
  digitalWrite(PB1, HIGH);
}

void loop() {
  digitalWrite(PB1, HIGH);
}