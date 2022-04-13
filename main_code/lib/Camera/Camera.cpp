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
        if ((blueAngle < 90 || blueAngle > 270) &&
            (yellowAngle > 90 || yellowAngle < 270)) {
            side = facingBlue;
        } else if ((yellowAngle < 90 || yellowAngle > 270) &&
                   (blueAngle > 90 || blueAngle < 270)) {
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

    oppGoalVec = Point(oppGoalAngle, oppGoalDist * 10);
    ownGoalVec = Point(ownGoalAngle, ownGoalDist * 10);

    // vector pointing to the centre of the field
    centreVector = oppGoalVec + ownGoalVec;

    centreVector /= 2;

    // vector pointing to front of field
    frontVector = oppGoalVec - centreVector;

}

void Camera::update() {
    if (read()) {
        process();
    }
}

void Camera::printData(int timeOut) {
    if (millis() - lastPrintTime > timeOut) {
        lastPrintTime = millis();
        String obj[] = {"Ball", "Blue", "Yellow"};
        String vals[] = {"Angle", "Dist"};
        if (side == facingBlue) {
            Serial.println("FACING BLUE");
        } else if (side == facingYellow) {
            Serial.println("FACING YELLOW");
        }
        if (ballVisible) {
            Serial.print("Ball Angle: ");
            Serial.print(ballAngle);
            Serial.print(" Ball Dist: ");
            Serial.print(ballDist);
        }
        if (blueVisible) {
            Serial.print(" Blue Angle: ");
            Serial.print(blueAngle);
            Serial.print(" Blue Dist: ");
            Serial.print(blueDist);
        }
        if (yellowVisible) {
            Serial.print(" Yellow Angle: ");
            Serial.print(yellowAngle);
            Serial.print(" Yellow Dist: ");
            Serial.print(yellowDist);
        }
        Serial.println();

        if (blueVisible && yellowVisible) {
            Serial.print("Orientation: ");
            Serial.print(-frontVector.getAngle());
            Serial.print(" Angle to Centre: ");
            Serial.print(centreVector.getAngle());
            Serial.print(" Distance to Centre: ");
            Serial.print(centreVector.getDistance());
            Serial.println();
            Serial.print("X: ");
            Serial.print(-centreVector.x);
            Serial.print(" Y: ");
            Serial.println(-centreVector.y);
        }
    }
   
}
