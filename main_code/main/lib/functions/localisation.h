#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <declarations.h>

void updatePosition() {
    // TODO: combine camera data with TOF data
    // TODO: use light sensors to confirm robot's position along x-axis
    bbox.update(tof, lineData, moveData.rotation.val, camera);
    botCoords.x = bbox.x;
    botCoords.y = bbox.y;
    bbox.processTOFout();
}

// bool reachedPoint(Point target, int dist = 30) {
//     // if (camera.blueVisible && camera.yellowVisible){
//     //     Point moveVector = camera.centreVector + target;

//     //     return moveVector.getDistance() <= dist;
//     // } else {
//     Point moveVector = target - botCoords;
//     float x_axis = sin(deg2rad(moveVector.getAngle())) * bbox.Xconfidence;
//     float y_axis = cos(deg2rad(moveVector.getAngle())) * bbox.Yconfidence;
//     float moveAngle = polarAngle(y_axis, x_axis);

//     // confident in both x and y axis
//     return (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) < dist &&
//             abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) < dist);
//     //}
// }

bool pointOnLine(Point tmp) { return abs(tmp.x) >= 960; }

bool goTo(Point target, int distThresh = 50) {
    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point camVector, tofVector, pointVector;
    float moveSpeed, moveAngle, dist;

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

    moveSpeed = constrain(pointVector.getDistance() * 0.2, 30, 50);
    Serial.print("TOF: ");
    Serial.print(" Angle to target: ");
    Serial.print(moveAngle);
    Serial.print(" Distance to target: ");
    Serial.println(pointVector.getDistance());

    // Serial.println();
    // CAMERA CONFIDENCE CAN BE BASED ON NUMBER OF SEPERATE BLOBS DETECTED

    setMove(moveSpeed, moveAngle, 0);
    if (pointVector.getDistance() <= distThresh) {
        setMove(0, 0, 0);
        if (bbox.Xconfidence >= 0.9 && bbox.Yconfidence >= 0.9) distCnt++;
    } else {
        distCnt = 0;
    }
    return distCnt >= 100;
}

bool TOFtoPoint(int FBdist, int LRdist, int FB, int LR, int distThresh = 80) {
    // calc vector needed to reach desired tof readings (uses only 1 vertical
    // and 1 horizontal tof)
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