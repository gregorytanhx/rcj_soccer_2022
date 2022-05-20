#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <declarations.h>
#include <localisation.h>

void updateDribbler() {
    // use pwm to control dribbler
    // turn on dribbler if facing ball and within 50 cm
    // turn off dribbler if ball in possession, facing goal and within 50cm
    if (dribble) {
        analogWrite(DRIBBLER_PIN, DRIBBLER_UPPER_LIMIT);
    } else {
        analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    }
}
long kickTime = 0;
long kickInterval = 0;
bool stopKick = false;
void updateKick() {
    if (kick) {
        if (!kicked) {
            digitalWriteFast(KICKER_PIN, LOW);
            kickTime = millis();
            kicked = true;
            // Serial.println("kicked");
        } else {
            // solenoid sticks out for 100ms
            if (millis() - kickTime > 50) {
                digitalWriteFast(KICKER_PIN, HIGH);
                // Serial.println("stopped kicking");
                stopKick = true;
            }
            // if kicker is on, kick every 2 seconds
            if (stopKick && millis() - kickTime > 2000) {
                kicked = false;
                Serial.println("kicking again");
                stopKick = false;
            }
        }
    } else {
        digitalWriteFast(KICKER_PIN, HIGH);
    }
}

void updateBallData() {
    if (readLightGate() <= 20) lastGateTime = millis();
    ballData.captured = millis() - lastGateTime < 200;
    if (camera.ballVisible) {
        ballData.angle = camera.ballAngle;
        ballData.dist = camera.ballDist;
        lastBallTime = millis();
        relBall = Point(ballData.angle, ballData.dist * 10);
        absBall = relBall - camera.centreVector;

        ballData.x = absBall.x + 100;
        ballData.y = absBall.y;
        // Serial.print("Abs Ball Coords: ");
        // Serial.print(ballData.x);
        // Serial.print(" ");
        // Serial.println(ballData.y);
    } else if (millis() - lastBallTime < 3000 && camera.predBall) {
        // use predicted ball if ball was last seen less than one second ago

        ballData.angle = camera.predBallAngle;
        ballData.dist = camera.predBallDist;

        // do NOT update ball timer here
    } else if (bt.otherData.ballData.visible) {
        // find ball position based on other robot's data
        // derive angle and distance of ball relative to robot based on its
        // absolute coordinates

        // relative coordinates to other robot
        absBall = Point(bt.otherData.ballData.x, bt.otherData.ballData.y);
        botCoords = Point(0, 0);
        // relative coordinates to self
        relBall = absBall - botCoords;
        ballData.angle = mod(relBall.getAngle(), 360);
        ballData.dist = relBall.getDistance() / 10;
        // treat ball as visible
        ballData.visible = true;
        lastBallTime = millis();
        Serial.println("Using ball data from other bot");
        Serial.print("Ball Angle: ");
        Serial.print(ballData.angle);
        Serial.print(" Ball Dist: ");
        Serial.println(ballData.dist);
    }

    ballData.visible = millis() - lastBallTime < 500;
}

void updateLineControl() {
    // determine how to handle line when ball tracking
    if (moveData.speed.val < 70) {
        lineTrack = true;
        lineAvoid = false;
    } else {
        lineTrack = false;
        lineAvoid = true;
    }
    if (lineData.onLine &&
        abs(moveData.angle.val - lineData.initialLineAngle.val) > 90) {
        // stop line tracking if desired angle of movement is opposite of the
        // line
        lineTrack = false;
        lineAvoid = false;
    }
}

void trackBall() {
    float ballOffset, ballMult;
    // ballData.dist = max(ballData.dist, 0);
    if (ballData.angle < 180)
        ballOffset = fmin(ballData.angle * 1.0, 90);
    else
        ballOffset = max((ballData.angle - 360) * 1.0, -90);

    float factor = 1 - ballData.dist / 90;
    ballMult = fmin(1.2, 0.016 * exp(factor * 4.9));
   
    

    robotAngle = ballData.angle + ballMult * ballOffset;
   
    
    // if (ballData.angle <)
    // if (ballData.dist > 35)
    //     robotAngle = ballData.angle + ballOffset * 0.35;
    // else if (ballData.dist > 20)
    //     robotAngle = ballData.angle + ballOffset * 0.7;
    // else
    //     robotAngle = ballData.angle + ballOffset * 0.9;

    if (robotAngle < 20 || robotAngle > 340 ||
        (robotAngle > 160 && robotAngle < 200)) {
        runningSpeed = 80;
    } else {
        runningSpeed = 75;
    }

    // if ((ballData.angle < 20 || ballData.angle > 340) && ballData.dist < 25) {
    //     robotAngle = 0;
    //     runningSpeed = 60;
    // }

    // runningSpeed = 60;
    // ball directly next to robot

    Serial.print("Ball Angle: ");
    Serial.print(ballData.angle);
    Serial.print(" Ball Dist: ");
    Serial.print(ballData.dist);
    Serial.print(" Ball Offset: ");
    Serial.print(ballOffset);
    Serial.print(" Move Angle: ");
    Serial.println(mod(robotAngle + 360, 360));

}

void trackGoal(float goalAngle = -1.0) {
    if (goalAngle == -1.0) {
        //    goalAngle = mod(camera.oppGoalAngle - 10 + 360, 360);
        goalAngle = camera.oppGoalAngle;
    }
    float goalOffset;
    if (goalAngle < 180)
        goalOffset = fmin(goalAngle, 90);
    else
        goalOffset = max((goalAngle - 360), -90);

    robotAngle = goalAngle + 1.1 * goalOffset;
    runningSpeed = 75;
    Serial.print("Goal Angle: ");
    Serial.print(goalAngle);
    Serial.print(" Move Angle: ");
    Serial.println(robotAngle);
}

void angleCorrect(int target = 0) {
    moveData.rotation.val = cmpCorrection;
    moveData.angSpeed.val = 20;
}

void aimGoal() {
    float correction = (nonReflex(camera.oppGoalAngle) - 12) * 0.2;
    Serial.print(camera.oppGoalAngle);
    Serial.print(" ");
    Serial.print(correction);
    Serial.println();

    if (moveData.speed.val == 0)
        moveData.angSpeed.val = 30;
    else
        moveData.angSpeed.val = 15;
    moveData.rotation.val = correction;
}
void camAngleCorrect(int targetAng = 0) {
    if (camera.blueVisible && camera.yellowVisible) {
        // Serial.print("Orientation: ");
        // Serial.println(camera.frontVector.getAngle());
        moveData.rotation.val =
            camAngPID.update(camera.frontVector.getAngle() - targetAng);
        // Serial.print("Correction: ");
        // Serial.println(moveData.rotation.val);
        moveData.angSpeed.val = 15;
    } else {
        // Serial.println("FUCKKKKKK");
        moveData.rotation.val = 0;
    }
}

#endif