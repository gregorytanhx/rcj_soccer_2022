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
Motors motors;
PID lineTrackPID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);
float lastLineAngle = 0;

// void lineAvoidance() {
//   if (abs(lastLineAngle - lineData.lineAngle) >= 90) {
//     lineAngle = lastLineAngle;
//     // allow chord length to keep increasing as robot goes over centre of line
//     lineData.chordLength = 2 - lineData.chordLength;
//   }
//   // move at speed proportional to how far the robot is over the line
//   motors.setMove(80 * lineData.chordLength, lineData.lineAngle, 0);
// }

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("no");
}