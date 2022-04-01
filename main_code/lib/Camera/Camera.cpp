#include "Camera.h"

void Camera::begin() { CamSerial.begin(CAMERA_BAUD); }

bool Camera::read() {
    bool newData = false;
    while (CamSerial.available() >= CAMERA_PACKET_SIZE) {
        newData = true;
        uint8_t syncByte = CamSerial.read();
        if (syncByte == CAMERA_SYNC_BYTE) {
            for (int i = 0; i < CAMERA_PACKET_SIZE - 1; i++) {
                buffer.b[i] = CamSerial.read();
            }
        }

        ballAngle = buffer.vals[0];
        ballDist = buffer.vals[1];
        blueAngle = buffer.vals[2];
        blueDist = buffer.vals[3];
        yellowAngle = buffer.vals[4];
        yellowDist = buffer.vals[5];
    }
    return newData;
}


void Camera::process() {
    blueVisible = blueAngle != 500;
    yellowVisible = yellowAngle != 500;
    ballVisible = ballAngle != 500;

    // change angle range to 180 to -180
    ballAngle = nonReflex(ballAngle);
    blueAngle = nonReflex(blueAngle);
    yellowAngle = nonReflex(yellowAngle);

    // set side only once
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
        oppVisible = blueVisible;
        ownVisible = yellowVisible;
    } else {
        oppAngle = yellowAngle;
        oppDist = yellowDist;
        ownAngle = blueAngle;
        ownDist = blueDist;
        oppVisible = yellowVisible;
        ownVisible = blueVisible;
    }

    oppGoalVec = Point(oppAngle, oppDist);
    ownGoalVec = Point(ownAngle, ownDist);

    // vector pointing to the centre of the field
    centreVector = oppGoalVec + ownGoalVec;
    centreVector.distance /= 2;

    // vector pointing to front of field
    frontVector = oppGoalVec - centreVector;
}

void Camera::update() {
    if (read()) {
        process();
    }
}