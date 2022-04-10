#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <declarations.h>

void updatePosition() {
    // TODO: combine camera data with TOF data
    // TODO: use light sensors to confirm robot's position along x-axis
    lineData.onLine = false;
    bbox.update(tof, lineData, moveData.rotation.val, camera);
    botCoords.x = bbox.x;
    botCoords.y = bbox.y;
}

bool reachedPoint(Point target, int dist = 30) {
    // if (camera.blueVisible && camera.yellowVisible){
    //     Point moveVector = camera.centreVector + target; 
    
    //     return moveVector.getDistance() <= dist;
    // } else {
        Point moveVector = target - botCoords;
        float x_axis = sin(deg2rad(moveVector.getAngle())) * bbox.Xconfidence;
        float y_axis = cos(deg2rad(moveVector.getAngle())) * bbox.Yconfidence;
        float moveAngle = polarAngle(y_axis, x_axis);

        // confident in both x and y axis
        return (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) <
                    dist &&
                abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) < dist);
    //}
   
}


bool pointOnLine(Point tmp) { return abs(tmp.x) >= 960; }

void goTo(Point target) {
    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point moveVector;
    float moveSpeed, moveAngle;
    
    // if (camera.oppGoalVisible && camera.ownGoalVisible) {
    //     // go to point based on vector calculations from camera
        
    //     moveVector = camera.centreVector + target;
    //     moveAngle = moveVector.getAngle();
    //     moveSpeed = constrain(moveVector.getDistance() * 0.1, 30, 60);
    //     // Serial.print("Camera: ");
    //     // Serial.print("Angle to target: ");
    //     // Serial.print(moveVector.getAngle());
    //     // Serial.print(" Distance to target: ");
    //     // Serial.print(moveVector.getDistance());
    //     // Serial.println();
    //     // Serial.print(target.getAngle());
    //     // Serial.print(" ");
    //     // Serial.println(target.getDistance());
    // } 
    
    // else {
        // go to point based on robot's coordinates
        moveVector = target - botCoords;

        // get initial gauge for speed
        moveSpeed = constrain(moveVector.getDistance() * 0.1, 30, 50);
      
        bbox.print();
        // if using coordinates, adjust angle based on confidence in position
        // along each axis eg. if x-axis is completely blocked, move along
        // y-axis until robot is not blocked

        if (bbox.Xconfidence < 0.4) bbox.Xconfidence = 0;
        if (bbox.Yconfidence < 0.4) bbox.Yconfidence = 0;

        // adjust x/y ratio based on confidence to produce new weighted angle
        float x_axis = sin(deg2rad(moveVector.getAngle())) * bbox.Xconfidence;
        float y_axis = cos(deg2rad(moveVector.getAngle())) * bbox.Yconfidence;
        moveAngle = polarAngle(y_axis, x_axis);

        float Xspeed = abs(moveSpeed * sin(deg2rad(moveAngle)));
        float Yspeed = abs(moveSpeed * cos(deg2rad(moveAngle)));

        if (abs(moveVector.getDistance() * sin(deg2rad(moveAngle))) < 30) {
            Xspeed = 0;
        }
        if (abs(moveVector.getDistance() * cos(deg2rad(moveAngle))) < 30) {
            Yspeed = 0;
        }

        moveSpeed = constrain(sqrt(Xspeed * Xspeed + Yspeed * Yspeed), 30, 55);
        Serial.print("TOF: ");
        Serial.print(" Angle to target: ");
        Serial.print(moveAngle);
        Serial.print(" Distance to target: ");
        Serial.print(moveVector.getDistance());

        Serial.println();
        // Serial.print("X Dist");
        // Serial.print(moveVector.getDistance() * sin(deg2rad(moveAngle)));
        // Serial.print(" Y Dist");
        // Serial.println(moveVector.getDistance() * cos(deg2rad(moveAngle)));
        // Serial.print("X Speed");
        // Serial.print(Xspeed);
        // Serial.print(" Y Speed");
        // Serial.println(Yspeed);
    //}






    // if (pointOnLine (target)) {
    //     lineTrack = true;
    //     if (lineData.onLine) {
    //         // can afford to move at faster speed if line tracking
    //         moveSpeed = 70;
    //     }
    // }
    setMove(moveSpeed, moveAngle, 0);
    if (moveVector.getDistance() <= 30) {
        setMove(0, 0, 0);
    } 
   
}



#endif