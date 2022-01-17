#include "Camera.h"

void Camera::init() {
  Serial1.begin(CAMERA_BAUD);
}

void Camera::read() {
  while (Serial1.available() >= CAMERA_PACKET_SIZE) {
    uint8_t syncByte = Serial1.read();
    if (syncByte == CAMERA_SYNC_BYTE) {
      for (int i = 0; i < CAMERA_PACKET_SIZE - 1; i++) {
        buffer.b[i] = Serial1.read();
      }
    }

    ballAngle = buffer.vals[0];
    ballPixelDist = buffer.vals[1];
    blueAngle = buffer.vals[2];
    bluePixelDist = buffer.vals[3];
    yellowAngle = buffer.vals[4];
    yellowPixelDist = buffer.vals[5];
  }
}

float cmDist(float pixelDist) {
  // TO BE DONE
  return pixelDist * 10;
}

void Camera::process() {
  if (blueAngle >= 0 && bluePixelDist >= 0) {
    blueVisible = true;
  } else {
    blueVisible = false;
  }
  if (yellowAngle >= 0 && yellowPixelDist >= 0) {
    yellowVisible = true;
  } else {
    yellowVisible = false;
  }
  if (ballAngle >= 0 && ballPixelDist >= 0) {
    ballVisible = true;
  } else {
    ballVisible = false;
  }

  // change angle range to 180 to -180
  ballAngle = nonReflex(ballAngle);
  blueAngle = nonReflex(blueAngle);
  yellowAngle = nonReflex(yellowAngle);

  ballDist = cmDist(ballPixelDist);
  yellowDist = cmDist(yellowPixelDist);
  blueDist = cmDist(bluePixelDist);

  if ((blueVisible && yellowVisible) && side == unset) {
    if (abs(blueAngle) <= 90 && abs(yellowAngle) >= 90) {
      side = facingBlue;
    } else if (abs(yellowAngle) <= 90 && abs(blueAngle) >= 90) {
      side = facingYellow;
    }
  }

  if (side == facingBlue) {
    ownAngle = yellowAngle;
    ownDist = yellowDist;
    oppAngle = blueAngle;
    oppDist = blueDist;
  } else if (side == facingBlue) {
    oppAngle = yellowAngle;
    oppDist = yellowDist;
    ownAngle = blueAngle;
    ownDist = blueDist;
  }
  
}

