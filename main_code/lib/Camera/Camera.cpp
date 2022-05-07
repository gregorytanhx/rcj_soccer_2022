#include "Camera.h"

void Camera::begin() {
    CamSerial.begin(CAMERA_BAUD);
    // while (side == unset) {
    //     read();
    //     // set side only once
    //     if ((blueVisible && yellowVisible)) {
    //         if ((blueAngle < 90 || blueAngle > 270) &&
    //             (yellowAngle > 90 || yellowAngle < 270)) {
    //             side = facingBlue;
    //         } else if ((yellowAngle < 90 || yellowAngle > 270) &&
    //                    (blueAngle > 90 || blueAngle < 270)) {
    //             side = facingYellow;
    //         }
    //     }
    // }
}

bool Camera::read() {
    newData = false;
    float tmpArray[8];
    while (CamSerial.available() >= CAMERA_PACKET_SIZE) {
        newData = true;
        uint8_t syncByte = CamSerial.read();
        if (syncByte == CAMERA_SYNC_BYTE) {
            for (int i = 0; i < CAMERA_PACKET_SIZE - 1; i++) {
                buffer.b[i] = CamSerial.read();
            }
        }
        for (int i = 0; i < 8; i++) {
            tmpArray[i] = (float)buffer.vals[i] / 100;
        }

        ballAngle = tmpArray[0];
        ballDist = tmpArray[1];
        predBallAngle = tmpArray[2];
        predBallDist = tmpArray[3];
        blueAngle = tmpArray[4];
        blueDist = tmpArray[5];
        yellowAngle = tmpArray[6];
        yellowDist = tmpArray[7];
    }
    return newData;
}

void Camera::process() {
    blueVisible = blueAngle != 500;
    yellowVisible = yellowAngle != 500;
    ballVisible = ballAngle != 500;
    predBall = predBallAngle != 500;

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
        Serial.print("Camera time: ");
        Serial.println(micros() - lastReadTime);
       
        process();
        lastReadTime = micros();
    }
}

void Camera::printData(int timeOut) {
    lastPrintTime = millis();

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
    if (predBall) {
        Serial.print("Predicted Ball Angle: ");
        Serial.print(predBallAngle);
        Serial.print(" Predicted Ball Dist: ");
        Serial.print(predBallDist);
    }
    Serial.println();
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
     if (oppGoalVisible) {
        Serial.print(" Opp Angle: ");
        Serial.print(oppGoalAngle);
        Serial.print(" Opp Dist: ");
        Serial.print(oppGoalDist);
    }
    if (ownGoalVisible) {
        Serial.print(" Own Angle: ");
        Serial.print(ownGoalAngle);
        Serial.print(" Own Dist: ");
        Serial.print(ownGoalDist);
    }
    Serial.println();

    if (blueVisible && yellowVisible) {
        Serial.print("Orientation: ");
        Serial.print(-frontVector.getAngle());
        Serial.print(" Angle to Centre: ");
        Serial.print(nonReflex(centreVector.getAngle()));
        Serial.print(" Distance to Centre (cm): ");
        Serial.print(centreVector.getDistance() / 10);
        Serial.println();
        Serial.print("X: ");
        Serial.print(-centreVector.x);
        Serial.print(" Y: ");
        Serial.println(-centreVector.y);
    }
}
