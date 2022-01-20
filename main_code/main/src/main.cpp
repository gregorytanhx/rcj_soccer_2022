#include <Arduino.h>
#include <Config.h>
#include <Common.h>
#include <Light.h>
#include <Motor.h>
#include <PID.h>
#include <Point.h>
#include <Pins.h>
#include <Wire.h>
#include <Adafruit_BNO055_t4.h>
#include <utility/imumaths.h>

Light light;
LineData lineData;
MoveData moveData;
Motors motors;
PID lineTrackPID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);
float lastLineAngle = 0;
bool lineTrack = false;

// void lineAvoidance() {
//   if (abs(lastLineAngle - lineData.lineAngle) >= 90) {
//     lineAngle = lastLineAngle;
//     // allow chord length to keep increasing as robot goes over centre of line
//     lineData.chordLength = 2 - lineData.chordLength;
//   }
//   // move at speed proportional to how far the robot is over the line
//   motors.setMove(80 * lineData.chordLength, lineData.lineAngle, 0);
// }


void setMove(float speed, float angle, float rotation) {
  moveData.speed.val = speed;
  moveData.angle.val = angle;
  moveData.rotation.val = rotation;
}

void sendLayer1() {
  Serial.write(LAYER1_REC_SYNC_BYTE);
  Serial.write(moveData.speed.b, 4);
  Serial.write(moveData.angle.b, 4);
  Serial.write(moveData.rotation.b, 4);
  Serial.write(lineTrack);
}

// void lineTrack(float target) {
//   float angle = nonReflex(line.getClosestAngle(target));
//   // use PID to control speed of correction
//   float correction = lineTrackPID.update(angle - target);
  
//   motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
// }

// void getCameraCoords() {
//   Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
//   Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

//   int vecX = (oppGoalVec.x + ownGoalVec.x) / 2;
//   int vecY = (oppGoalVec.y + oppGoalVec.y) / 2;

//   Point centre(vecX, vecY);

// }

void readLayer1() {  
  while (Serial1.available() > LAYER1_SEND_PACKET_SIZE) {
    Serial.readBytes(lineData.lineAngle.b, 4);
    //Serial.readBytes(lineData.closestAngle.b, 4);
    Serial.readBytes(lineData.chordLength.b, 4);
    lineData.onLine = Serial.read();
  }

}

void setup() {
  Serial.begin(9600);
  Serial1.begin(2000000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("1");
}