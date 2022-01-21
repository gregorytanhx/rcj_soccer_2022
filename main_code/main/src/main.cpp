#include <Arduino.h>
#include <Config.h>
#include <Common.h>
#include <Light.h>
#include <Motor.h>
#include <Bluetooth.h>
#include <PID.h>
#include <Point.h>
#include <Pins.h>
#include <Wire.h>
#include <Adafruit_BNO055_t4.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

Light light;
LineData lineData;
MoveData moveData;
Motors motors;
float lastLineAngle = 0;
bool lineTrack = false;
bool hasBall = false;
TOFBuffer tof;
IMU imu;
float heading;

Timer kickerTimer(500);

Role role = striker; 

void dribble() {
  analogWrite(DRIBBLER_PIN, 64);
}

void kick() {
  if (kickerTimer.timeHasPassed()) {
    digitalWrite(KICKER_PIN, HIGH);
  }  
}


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

void readLayer1() {
  while (Serial1.available() >= LAYER1_SEND_PACKET_SIZE) {
    uint8_t syncByte = Serial1.read();
    if (syncByte == LAYER1_REC_SYNC_BYTE) {
      for (int i = 0; i < 4; i++) {
        lineData.lineAngle.b[i] = Serial1.read();
      }
      for (int i = 0; i < 4; i++) {
        lineData.chordLength.b[i] = Serial1.read();
      }
      lineData.onLine = Serial1.read();
    }
  }

void readLayer4()  {
  while (Serial2.available() >= LAYER4_PACKET_SIZE) {
    uint8_t syncByte = Serial2.read();
    if (syncByte == LAYER4_SYNC_BYTE) {
      for (int i = 0; i < LAYER4_PACKET_SIZE - 1; i++){
        tof.b[i] = Serial2.read();
      }
    }
  }
}

void processTOF() {
  // TODO
}

void trackBall() {
  // TODO
}

void aimGoal() {
  // TODO
}

void defend() {
  // TODO
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

  void setup() {


    Serial.begin(9600);
    Serial1.begin(2000000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    camera.init();
    imu.init();

    pinMode(KICKER_PIN, OUTPUT);
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWriteFrequency(DRIBBLER_PIN, 1000);
    analogWrite(DRIBBLER_PIN, 32);
    delay(DRIBBLER_WAIT);
  }

  void loop()  {
    // put your main code here, to run repeatedly:
    camera.read();
    camera.process();
    heading = imu.read();
    readLayer4();

    if (analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH) {
      hasBall = true;
    } 
    
    if (role == striker) {
      if (hasBall) {
        aimGoal();
      }  else {
        trackBall();
      }
    }
    
    sendLayer1();
    readLayer1();
   

  }