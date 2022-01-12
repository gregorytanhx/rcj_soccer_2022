#include <Arduino.h>


#define MOTOR_PACKET_SIZE 13
#define MOTOR_SYNC_BYTE 2

Light light;
Motors motors;

LineData lineData;
motorBuffer buffer;

void sendData() {
  //send light vals
  
}

void receiveData() {
  //receive motor data
  while (Serial1.available() >= MOTOR_PACKET_SIZE) {
    uint8_t syncByte = Serial1.read();
    if (syncByte == MOTOR_SYNC_BYTE) {
      for (int i = 0; i < MOTOR_PACKET_SIZE - 1; i++) {
        motorBuffer.b[i] = Serial1.read();
      }
    }
  }
}

//handle line avoidance directly through stm32
void setup() {
  light.init();
  motors.init()
  pinMode(PB1, OUTPUT);
  digitalWrite(PB1, HIGH);
  Serial.begin(9600);
}

void loop() {
  

}