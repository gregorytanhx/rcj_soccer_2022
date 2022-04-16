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

bool goTo(Point target, int distThresh = 40) {
    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point camVector, tofVector;
    float moveSpeed, moveAngle, dist;

    // else {
    // go to point based on robot's coordinates
    tofVector = target - botCoords;
    dist = tofVector.getDistance();
    // get initial gauge for speed
    moveSpeed = constrain(tofVector.getDistance() * 0.1, 30, 50);

    // bbox.print();
    //  if using coordinates, adjust angle based on confidence in position
    //  along each axis eg. if x-axis is completely blocked, move along
    //  y-axis until robot is not blocked

    if (bbox.Xconfidence < 0.4) bbox.Xconfidence = 0;
    if (bbox.Yconfidence < 0.4) bbox.Yconfidence = 0;

    // adjust x/y ratio based on confidence to produce new weighted angle
    float x_axis = sin(deg2rad(tofVector.getAngle())) * bbox.Xconfidence;
    float y_axis = cos(deg2rad(tofVector.getAngle())) * bbox.Yconfidence;
    moveAngle = polarAngle(y_axis, x_axis);

    float Xspeed = abs(moveSpeed * sin(deg2rad(moveAngle)));
    float Yspeed = abs(moveSpeed * cos(deg2rad(moveAngle)));

    if (abs(tofVector.getDistance() * sin(deg2rad(moveAngle))) < 30) {
        Xspeed = 0;
    }
    if (abs(tofVector.getDistance() * cos(deg2rad(moveAngle))) < 30) {
        Yspeed = 0;
    }

    moveSpeed = constrain(sqrt(Xspeed * Xspeed + Yspeed * Yspeed), 30, 50);
    // Serial.print("TOF: ");
    // Serial.print(" Angle to target: ");
    // Serial.print(moveAngle);
    // Serial.print(" Distance to target: ");
    // Serial.print(tofVector.getDistance());

    // Serial.println();

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