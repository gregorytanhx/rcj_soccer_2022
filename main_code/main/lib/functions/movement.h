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

void updateKick() {
    if (kick) {
        digitalWriteFast(KICKER_PIN, LOW);
        if (!kicked) {
            kickerTimer.update();
        }
        kicked = true;
    }
    if (kicked && kickerTimer.timeHasPassed(false)) {
        digitalWriteFast(KICKER_PIN, HIGH);
        kicked = false;
    }
}


void updateBallData() {
    
    if (readLightGate()) lastGateTime = millis();
    ballData.captured = millis() - lastGateTime < 100;
    if (camera.ballVisible) {
        relBallCoords = Point(camera.ballAngle, camera.ballDist);
        absBallCoords = relBallCoords + botCoords;
        ballData.angle = camera.ballAngle;
        ballData.dist = camera.ballDist;
        ballData.x = absBallCoords.x;
        ballData.y = absBallCoords.y;
        lastBallTime = millis();

    } else if (bt.otherData.ballData.visible) {
        // find ball position based on other robot's data
        // derive angle and distance of ball relative to robot based on its
        // absolute coordinates

        absBallCoords = Point(bt.otherData.ballData.x, bt.otherData.ballData.y);
        relBallCoords = absBallCoords - botCoords;
        ballData.angle = relBallCoords.getAngle();
        ballData.dist = relBallCoords.getDistance();
        // treat ball as visible
        ballData.visible = true;
        lastBallTime = millis();
    }
    ballData.visible = millis() - lastBallTime < 500;
}

void updateLineControl() {
    // determine how to handle line
    if (moveData.speed.val < 80) {
        lineTrack = true;
        lineAvoid = false;
    } else {
        lineTrack = false;
        lineAvoid = true;
    }
    if (lineData.onLine &&
        abs(moveData.angle.val - lineData.initialLineAngle.val > 90)) {
        // stop line tracking if desired angle of movement is opposite of the
        // line
        lineTrack = false;
        lineAvoid = true;
    }
}

void trackBall() {
    float ballOffset;
    if (ballData.angle < 180)
        ballOffset = fmin(ballData.angle * 0.96, 90);
    else
        ballOffset = max((360 - ballData.angle) * 0.96, -90);

    float factor = 1 - ballData.dist / 520;
    float ballMult = fmin(1, 0.0134 * exp(factor * 2.6));

    robotAngle = ballData.angle + ballMult * ballOffset;
    setMove(60, robotAngle, 0);
}

void trackGoal(float goalAngle = -1.0) {
    if (goalAngle == -1.0) goalAngle = camera.oppGoalAngle;
    float goalOffset;
    if (goalAngle < 180)
        goalOffset = fmin(goalAngle, 90);
    else
        goalOffset = max((360 - goalAngle), -90);

    float goalMult = 1.5;
    robotAngle = goalAngle + goalMult * goalOffset;
    setMove(60, robotAngle, 0);
}


void angleCorrect(int targetAng = 0) { 
    moveData.rotation.val = cmpPID.update(cmp.readQuat() - targetAng); 
}

void camAngleCorrect(int targetAng = 0) {

    moveData.rotation.val = cmpPID.update(camera.frontVector.getAngle() - targetAng);
}

#endif