#include <Arduino.h>

uint8_t i = 0;
void setup() {
  pinMode(PA4, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(PA4, LOW);
  Serial.println(i);
  i++;
}