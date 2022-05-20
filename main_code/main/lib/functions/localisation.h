#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <declarations.h>

void updatePosition() {
    // TODO: combine camera data with TOF data
    // TODO: use light sensors to confirm robot's position along x-axis
    bbox.update(tof, lineData, heading, camera);
    botCoords.x = bbox.x;
    botCoords.y = bbox.y;
}

void slowDown() {
    bool tmp = false;
    // int shortestDist = 999;

    // make speed proportional to axis angle
    float Xaxis = sin(deg2rad(moveData.angle.val));
    float Yaxis = cos(deg2rad(moveData.angle.val));
    // confidence based slowdown??
    float newSpeed = 0.5 * (Xaxis * moveData.speed.val * bbox.Xconfidence +
                            Yaxis * moveData.speed.val * bbox.Yconfidence);
    Serial.print("Confidence based speed: ");
    Serial.println(newSpeed);
    // if (abs(botCoords.x) > 300) tmp = true;

    // implement slowdown based on robotangle

    // if (camera.ownGoalDist < 48 || camera.oppGoalDist < 48)
    //     moveData.speed.val = 60;
    // else
    bool minSpeed = false;
    for (int i = 1; i < 4; i++) {
        
        if (bbox.tofVals[i] < 650 && !bbox.tofFlag[i]) {
            if (bbox.tofVals[i] <= 400) {
                minSpeed = true;
            }
               
            moveData.speed.val = 50;
        }
    }
    if (minSpeed) moveData.speed.val = 30;
   
}

void avoidLine() {
    lineAvoid = true;
    int lineStop;
    int outAngle = bbox.processTOFout();
    Serial.print("OUT");
    Serial.print(" Angle: ");
    Serial.println(outAngle);
    // strict avoidance
    if (lineData.onLine) {
        lastOutTime = millis();
        // if (outAngle >= 0) {
        //     lineAvoid = false;
        //     setMove(moveData.speed.val, outAngle, 0);
        // }

        if (previouslyIn && lineCnt <= 3) lineCnt++;
        previouslyIn = false;
        Serial.println(previouslyIn);
        // if (lineCnt > 3) {
        //     if (millis() - lastInTime > 500) {
        //         if (!lineStop) {
        //             lineStop = true;
        //             outBallAngle = ballData.angle;
        //             outBallDist = ballData.dist;
        //         }
        //         if (millis() - lastInTime > 2000 ||
        //             (ballData.visible && (abs(outBallAngle - ballData.angle))
        //             >
        //                 10 || abs(outBallDist - ballData.dist) > 20) ||
        //             !ballData.visible) {
        //             lineStop = 0;
        //             lineAvoid = true;
        //             if (outAngle >= 0) {
        //                 lineAvoid = false;
        //                 Serial.print("OUT");
        //                 Serial.print(" Angle: ");
        //                 Serial.println(outAngle);
        //                 setMove(moveData.speed.val, outAngle, 0);
        //             }
        //         } else {
        //             Serial.println("STOP");
        //             lineAvoid = false;
        //             setMove(0, 0, 0);
        //             dribble = false;
        //         }
        //     } else {
        //         Serial.println("STOP");
        //         lineAvoid = false;
        //         setMove(0, 0, 0);
        //         dribble = false;
        //     }
        // } else {
        if (outAngle >= 0) {
            lineAvoid = false;
            Serial.print("OUT");
            Serial.print(" Angle: ");
            Serial.println(outAngle);
            setMove(60, outAngle, 0);
        }
        // }
    } else {
        lastInTime = millis();
        previouslyIn = true;
        if (millis() - lastOutTime > 800) {
            lineCnt = 0;
            lineStop = false;
        }
    }

    if (millis() - lastOutTime < 200) {
        if (outAngle >= 0) {
            // lineAvoid = false;

            setMove(60, outAngle, 0);
        }
    }

    // lineAvoid = true;
    // lineTrack = false;
    // incorporate line tracking

    // if (lineData.onLine) {
    //     if (previouslyIn && lineCnt <= 3) lineCnt++;
    //     previouslyIn = false;
    //     lastLineTime = millis();
    //     if (lineCnt > 3 && attackState == chasingBall) {
    //         // after avoiding line for 3 times
    //         lineAvoid = false;

    //         if (millis() - lastInTime < 500) {
    //             // stop on line for 0.5s first
    //             setMove(0, 0, 0);
    //             outBallAngle = outBallAngle = ballData.angle;
    //         }
    //         if (angleDiff(outBallAngle, ballData.angle) > 40 || millis() -
    //         lastInTime > 5000 || !ballData.visible) {
    //             lineTrack = false;
    //             lineCnt = 0;
    //         } else {
    //             lineTrack = true;
    //             float dist = lineData.chordLength.val > 1 ?
    //             lineData.chordLength.val - 1 : 1 - lineData.chordLength.val;
    //             setMove(max(dist * 60, 35), robotAngle, 0);
    //         }
    //     }

    //     else {
    //         if (bbox.outAngle >= 0) {
    //             lineAvoid = false;
    //             Serial.print("OUT");
    //             Serial.print(" Angle: ");
    //             Serial.println(bbox.outAngle);
    //             setMove(runningSpeed, bbox.outAngle, 0);
    //         } else {
    //             lineAvoid = true;
    //         }
    //     }
    // } else {
    //     lastInTime = millis();
    //     previouslyIn = true;
    // }
}

// bool reachedPoint(Point target, int dist = 30) {
//     // if (camera.blueVisible && camera.yellowVisible){
//     //     Point moveVector = camera.centreVector + target;

//     //     return moveVector.getDistance() <= dist;
//     // } else {
//     Point moveVector = target - botCoords;
//     float x_axis = sin(deg2rad(moveVector.getAngle())) *
//     bbox.Xconfidence; float y_axis = cos(deg2rad(moveVector.getAngle()))
//     * bbox.Yconfidence; float moveAngle = polarAngle(y_axis, x_axis);

//     // confident in both x and y axis
//     return (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) <
//     dist &&
//             abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) <
//             dist);
//     //}
// }

bool pointOnLine(Point tmp) { return abs(tmp.x) >= 960; }

bool goTo(Point target, int distThresh = 50) {
    if (millis() - lastCoordTime > 1000) {
        coordPIDX.resetIntegral();
        coordPIDY.resetIntegral();
    }
    lastCoordTime = millis();

    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point camVector, tofVector, pointVector;
    float moveSpeed, moveAngle, dist, Xspeed, Yspeed;

    // else {
    // go to point based on robot's coordinates
    pointVector = target - botCoords;
    // bbox.print();
    //  if using coordinates, adjust angle based on confidence in position
    //  along each axis eg. if x-axis is completely blocked, move along
    //  y-axis until robot is not blocked

    // adjust x/y ratio based on confidence to produce new weighted angle
    pointVector.x *= bbox.Xconfidence;
    pointVector.y *= bbox.Yconfidence;
    moveAngle = pointVector.getAngle();

    Xspeed = coordPIDX.update(abs(pointVector.x));
    Yspeed = coordPIDY.update(abs(pointVector.y));
    if (abs(pointVector.x) < distThresh / 2) Xspeed = 0;
    if (abs(pointVector.y) < distThresh / 2) Yspeed = 0;
    moveSpeed = Xspeed + Yspeed;
    Serial.print("TOF: ");
    Serial.print(" Angle to target: ");
    Serial.print(moveAngle);
    Serial.print(" Distance to target: ");
    Serial.print(pointVector.getDistance());
    Serial.print(" Speed: ");
    Serial.println(moveSpeed);

    // Serial.println();
    // CAMERA CONFIDENCE CAN BE BASED ON NUMBER OF SEPERATE BLOBS DETECTED

    // if (abs(pointVector.x) < distThresh / 2) Xspeed = 0;
    // if (abs(pointVector.y) < distThresh / 2) Yspeed = 0;

    robotAngle = moveAngle;
    runningSpeed = min(80, moveSpeed);
    if (pointVector.getDistance() <= distThresh) {
        distCnt++;
    } else {
        distCnt = 0;
    }
    return distCnt >= 500;
}

bool TOFtoPoint(int FBdist, int LRdist, int FB, int LR, int distThresh = 80) {
    // calc vector needed to reach desired tof readings (uses only 1
    // vertical and 1 horizontal tof)
    int tmpX, tmpY;
    if (FB == 0)
        tmpY = bbox.tofVals[FB] - FBdist;
    else
        tmpY = FBdist - bbox.tofVals[FB];
    if (LR == 1)
        tmpX = tof.vals[LR] - LRdist;
    else
        tmpX = LRdist - bbox.tofVals[LR];

    Point moveVector(tmpX, tmpY);
    float moveSpeed = constrain(moveVector.getDistance() * 0.2, 30, 50);
    float moveAngle = moveVector.getAngle();
    setMove(moveSpeed, moveAngle, 0);
    Serial.print("fb ");
    Serial.print(FB);
    Serial.print(" lr ");
    Serial.println(LR);
    Serial.print(tmpX);
    Serial.print(" ");
    Serial.print(tmpY);
    Serial.print(" ");
    Serial.print(moveVector.getAngle());
    Serial.print(" ");
    Serial.println(moveVector.getDistance());
    if (moveVector.getDistance() < distThresh) {
        setMove(0, 0, 0);
    }
    return moveVector.getDistance() < distThresh;
}

bool CamToPoint(Point target, int distThresh = 90) {
    Point camVector;
    float moveSpeed, moveAngle, dist;
    if (camera.oppGoalVisible && camera.ownGoalVisible) {
        // go to point based on vector calculations from camera
        camVector = camera.centreVector + target;
        moveAngle = camVector.getAngle();
        // moveAngle = angleAverage(camVector.getAngle(), moveAngle);
        //  compensate for orientation
        moveAngle = nonReflex(moveAngle) - camera.frontVector.getAngle();
        moveSpeed = constrain(camVector.getDistance() * 0.1, 30, 50);
        // dist = (camVector.getDistance() + dist) / 2 ;
        dist = camVector.getDistance();

        Serial.print("X: ");
        Serial.print(-camera.centreVector.x);
        Serial.print(" Y: ");
        Serial.println(-camera.centreVector.y);
        // Serial.print("Camera: ");
        Serial.print("Angle to target: ");
        Serial.print(" ");
        Serial.print(moveAngle);

        Serial.print(" Distance to target: ");
        Serial.print(camVector.getDistance());
        Serial.println();
        Serial.print(target.getAngle());
        Serial.print(" ");
        Serial.println(target.getDistance());
    } else {
        moveSpeed = 0;
    }

    setMove(moveSpeed, moveAngle, 0);
    if (dist <= distThresh) {
        setMove(0, 0, 0);
        distCnt++;
    } else {
        distCnt = 0;
    }
    return distCnt >= 100;
}
#endif