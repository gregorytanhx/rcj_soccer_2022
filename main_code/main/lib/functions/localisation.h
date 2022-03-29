#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <declarations.h>

void updatePosition() {
    // TODO: combine camera data with TOF data
    // TODO: use light sensors to confirm robot's position along x-axis
    lineData.onLine = false;
    bbox.update(tof, lineData, moveData.rotation.val);
    botCoords.x = bbox.x;
    botCoords.y = bbox.y;
}

bool reachedPoint(Point target, int dist = 50) {
    Point moveVector = target - botCoords;
    float x_axis = sin(deg2rad(moveVector.getAngle())) * bbox.Xconfidence;
    float y_axis = cos(deg2rad(moveVector.getAngle())) * bbox.Yconfidence;
    float moveAngle = polarAngle(y_axis, x_axis);

    // confident in both x and y axis
    return (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) < dist &&
            abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) < dist);
}

bool pointOnLine(Point tmp) { return abs(tmp.x) >= 960 }

void goTo(Point target) {
    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point moveVector;
    float moveSpeed, moveAngle;
    if (camera.oppVisible && camera.ownVisible) {
        // go to point based on vector calculations from camera
        moveVector = camera.centreVector + target;
    } else {
        // go to point based on robot's coordinates
        moveVector = target - botCoords;
    }

    // get initial gauge for speed
    moveSpeed = fmax(coordPID.update(moveVector.getDistance()), 20);

    // if using coordinates, adjust angle based on confidence in position along
    // each axis eg. if x-axis is completely blocked, move along y-axis until
    // robot is not blocked

    if (!(camera.oppVisible && camera.ownVisible)) {
        if (bbox.Xconfidence < TOF_CONFIDENCE_THRESH) bbox.Xconfidence = 0;
        if (bbox.Yconfidence < TOF_CONFIDENCE_THRESH) bbox.Yconfidence = 0;

        float x_axis = sin(deg2rad(moveVector.getAngle())) * bbox.Xconfidence;
        float y_axis = cos(deg2rad(moveVector.getAngle())) * bbox.Yconfidence;
        moveAngle = polarAngle(y_axis, x_axis);

        float Xspeed = abs(moveSpeed * sin(deg2rad(moveAngle)));
        float Yspeed = abs(moveSpeed * cos(deg2rad(moveAngle)));

        if (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) < 50) {
            Xspeed = 0;
        }
        if (abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) < 50) {
            Yspeed = 0;
        }

        moveSpeed = sqrt(Xspeed * Xspeed + Yspeed * Yspeed);
        // Serial.print("X Dist");
        // Serial.print(moveVector.getDistance() * sin(deg2rad(moveAngle)));
        // Serial.print(" Y Dist");
        // Serial.println(moveVector.getDistance() * cos(deg2rad(moveAngle)));
        // Serial.print("X Speed");
        // Serial.print(Xspeed);
        // Serial.print(" Y Speed");
        // Serial.println(Yspeed);

    } else {
        moveAngle = moveVector.getAngle();
    }

    // Serial.print("Weighted Angle: ");
    // Serial.println(moveAngle);
    // Serial.print(" Angle: ");
    // Serial.print(moveVector.getAngle());
    // Serial.print(" Distance: ");
    // Serial.print(moveVector.getDistance());
    // Serial.print(" Speed: ");
    // Serial.println(moveSpeed);
    if pointOnLine (target) {
        lineTrack = true;
        if (lineData.onLine) {
            // can afford to move at faster speed if line tracking
            moveSpeed = 70;
        }
    }
    setMove(moveSpeed, moveAngle, 0);
}
#endif