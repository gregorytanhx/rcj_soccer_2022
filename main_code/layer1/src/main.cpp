#include <Arduino.h>


#define MOTOR_PACKET_SIZE 13
#define MOTOR_SYNC_BYTE 2

Light light;
Motors motors;
MoveData moveData;
LineData lineData;
motorBuffer buffer;
lineBuffer buffer;

bool linetrack = false;

void sendData() {
  //send light vals
  
}

void receiveData() {
  //receive teensy data
  while (Serial1.available() >= MOTOR_PACKET_SIZE) {
    uint8_t syncByte = Serial1.read();
    if (syncByte == MOTOR_SYNC_BYTE) {
      for (int i = 0; i < MOTOR_PACKET_SIZE - 1; i++) {
        motorBuffer.b[i] = Serial1.read();
      }
      linetrack = (bool) Serial1.read();
    }
  }
  moveData.speed = motorBuffer.vals[0];
  moveData.angle = motorBuffer.vals[1];
  moveData.rotation = motorBuffer.vals[2];
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
  light.read()
  receiveData()

}