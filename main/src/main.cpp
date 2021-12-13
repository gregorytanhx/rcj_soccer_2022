#include <Arduino.h>
#include <Light.h>
#include <Motor.h>
#include <PID.h>
#include <Pins.h>
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

Light light;
Motors motors;
lineTrackPID = PID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);

void lineAvoidance() {
  if (abs(lastLineAngle - lineAngle) >= 90) {
    lineAngle = lastLineAngle;
    // allow chord length to keep increasing as robot goes over centre of line
    light.chordLength = 2 - light.chordLength;
  }
  // move at speed proportional to how far the robot is over the line
  motors.setMove(80 * light.chordLength, light.lineAngle, 0);
}

void lineTrack(float target) {
  float angle = nonReflex(light.getClosestAngle(target));
  // use PID to control speed of correction
  float correction = lineTrackPID.update(angle - target);
  
  motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // put your mwwwain code here, to run repeatedly:
}