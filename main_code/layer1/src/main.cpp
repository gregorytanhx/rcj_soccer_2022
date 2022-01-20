#include <Arduino.h>
#include <Common.h>
#include <Pins.h>
#include <Light.h>
#include <Motors.h>
#include <PID.h>
#include <Config.h>

Light light;
Motors motors;
float speed;
float angle;
float rotation;
LineData lineData;
motorBuffer buffer;
float lastLineAngle;
float closestAngle;

PID lineTrackPID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);
bool lineTrack = false;

void sendData() {
  Serial1.write(LAYER1_SEND_SYNC_BYTE);
  Serial1.write(lineData.lineAngle.b, sizeof(lineData.lineAngle.b));
  //Serial1.write(lineData.closestAngle.b, sizeof(lineData.closestAngle.b));
  Serial1.write(lineData.chordLength.b, sizeof(lineData.chordLength.b));
  Serial1.write(lineData.onLine);
}

void receiveData() {
  //receive teensy data
  while (Serial1.available() >= MOTOR_PACKET_SIZE) {
    uint8_t syncByte = Serial1.read();
    if (syncByte == LAYER1_REC_SYNC_BYTE) {
      for (int i = 0; i < MOTOR_PACKET_SIZE - 1; i++) {
        motorBuffer.b[i] = Serial1.read();
      }
      lineTrack = (bool) Serial1.read();
    }
  }
  speed = motorBuffer.vals[0];
  angle = motorBuffer.vals[1];
  rotation = motorBuffer.vals[2];
  
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
  light.getLineData(LineData)
  receiveData();
  closestAngle = light.getClosestAngle(moveData.angle);
  sendData();
  

  if (lineData.onLine) {
    if (lineTrack) {
      float angle = nonReflex(line.getClosestAngle(lineData.closestAngle.closestAngle));
      // use PID to control speed of correction
      float correction = lineTrackPID.update(angle - closestAngle);

      motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
    } else{
      if (abs(lastLineAngle - lineData.lineAngle) >= 90) {
        // allow chord length to keep increasing as robot goes over centre of line
        lineData.chordLength = 2 - lineData.chordLength;
      }
      // line avoidance
      motors.setMove(speed * lineData.chordLength, lineData.lineAngle, 0)
    }
   
  } else {
    motors.setMove(speed, angle, rotation);
  }
  
  lastLineAngle = lineData.lineAngle;
  motors.moveOut();
}