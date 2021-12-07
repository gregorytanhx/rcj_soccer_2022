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

void lineAvoidance(){
  if (abs(lastLineAngle - lineAngle) >= 90){
    lineAngle = lastLineAngle;
    // allow chord length to keep increasing as robot goes over centre of line
    light.chordLength = 2 - light.chordLength;
  }
  // move at speed proportional to how far the robot is over the line
  setMove(80 * light.chordLength, light.lineAngle, 0);

}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
}