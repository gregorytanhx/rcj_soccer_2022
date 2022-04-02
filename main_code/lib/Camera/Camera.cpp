#include "Camera.h"

void Camera::begin() { CamSerial.begin(CAMERA_BAUD); }

bool Camera::read() {
    newData = false;
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

    // set side only once
    if ((blueVisible && yellowVisible) && side == unset) {
        if (abs(blueAngle) <= 90 && abs(yellowAngle) >= 90) {
            side = facingBlue;
        } else if (abs(yellowAngle) <= 90 && abs(blueAngle) >= 90) {
            side = facingYellow;
        }
    }

    if (side == facingBlue) {
        ownGoalAngle = yellowAngle;
        ownGoalDist = yellowDist;
        oppGoalAngle = blueAngle;
        oppGoalDist = blueDist;
        oppGoalVisible = blueVisible;
        ownGoalVisible = yellowVisible;
    } else {
        oppGoalAngle = yellowAngle;
        oppGoalDist = yellowDist;
        ownGoalAngle = blueAngle;
        ownGoalDist = blueDist;
        oppGoalVisible = yellowVisible;
        ownGoalVisible = blueVisible;
    }

    oppGoalVec = Point(oppGoalAngle, oppGoalDist);
    ownGoalVec = Point(ownGoalAngle, ownGoalDist);

    // vector pointing to the centre of the field
    centreVector = oppGoalVec + ownGoalVec;
    // use mm for localisation
    centreVector *= 10;
    centreVector /= 2;

    // vector pointing to front of field
    frontVector = oppGoalVec - centreVector;
}

void Camera::update() {
    if (read()) {
        process();
    }
}