#include "Light.h"

Light::Light() {
  sigA = SIG_A;
  sigB = SIG_B;

  pinsA[0] = mux_A1;
  pinsA[1] = mux_A2;
  pinsA[2] = mux_A3;
  pinsA[3] = mux_A4;

  pinsB[0] = mux_B1;
  pinsB[1] = mux_B2;
  pinsB[2] = mux_B3;
  pinsB[3] = mux_B4;
}

void Light::init() {
  pinMode(mux_A1, OUTPUT);
  pinMode(mux_A2, OUTPUT);
  pinMode(mux_A3, OUTPUT);
  pinMode(mux_A4, OUTPUT);

  pinMode(mux_B1, OUTPUT);
  pinMode(mux_B2, OUTPUT);
  pinMode(mux_B3, OUTPUT);
  pinMode(mux_B4, OUTPUT);

  for (int i = 0; i < 4; i++) {
    digitalWrite(pinsA[i], LOW);
    digitalWrite(pinsB[i], LOW);
  }
}
int Light::readMux(int channel, int controlPin[4], int sig) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  //read the value at the SIG pin
  delayMicroseconds(5);
  int val = analogRead(sig);
  //return the value
  return val;
}

void Light::read() {
  bool out = false;
  outSensors = 0;
  for (int i = 0; i < 16; i++){
    if (readMux(i, pinsA, sigA) > lightThresh[i]) {
      lineDetected[outSensors] = i;
      outSensors++;
    }
  }

  for (int i = 16; i < 32; i++) {
    if (readMux(i, pinsB, sigB) > lightThresh[i]) {
      lineDetected[outSensors] = i;
      outSensors++;
    }
  }
  onLine = outSensors > 0;
}

void Light::getLineData() {
    double vecX = 0;
    double vecY = 0;
    int chordStart = 0;
    int chordEnd = 0;
    double largestDiff = 0;
    
    
    // get line angle and chord length
    for (int i = 0; i < outSensors; i++) {
      double tmpAngle = deg2rad(i * 360 / 32);
      vecY += cos(tmpAngle);
      vecX += sin(tmpAngle);

      for (int j = i + 1; j < outSensors; j++) {
        float tmpDiff = angleDiff(i * 360 / 32, j * 360 / 32);
        if (tmpDiff > largestDiff) {
          chordStart = j;
          chordEnd = i;
          largestDiff = tmpDiff;
        }
      }

    }
  
    chordLength = norm(abs(chordStart - chordEnd), 15, 1);
    lineAngle = rad2deg(atan2(vecX, vecY));
    if (lineAngle < 0) lineAngle += 360;
    lineAngle = fmod(lineAngle + 180, 360);   
}

float Light::getClosestAngle(float target) {
  float closestAngle = 0;
  float minDiff = 360;
  for (int i = 0; i < 32; i++) {
    if (lightVals[i]) {
      float tmpAngle = deg2rad(i * 360 / 32);
      float diff = angleDiff(tmpAngle, target);
      if (diff < minDiff) {
        minDiff = diff;
        closestAngle = tmpAngle;
      }
    }
  }
  return closestAngle;
}