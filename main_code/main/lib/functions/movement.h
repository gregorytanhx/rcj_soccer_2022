#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <declarations.h>

void controlDribbler() {
    // use pwm to control dribbler
    bool dribble = ballData.dist <= BALL_DRIBBLE_THRESH;
    if (dribble || !dribblerTimer.timeHasPassed()) {
        analogWrite(DRIBBLER_PIN, DRIBBLER_UPPER_LIMIT);
        dribblerTimer.update();
    } else {
        analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    }
}

void updateKick() {
    if (kick) {
        digitalWriteFast(KICKER_PIN, LOW);
        kickerTimer.update();
    }
    if (kickerTimer.timeHasPassed(false)) {
        digitalWriteFast(KICKER_PIN, HIGH);
    }
}

void setMove(float speed, float angle, float rotation) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
}

void updateBallData() {
    ballData.visible = camera.ballVisible;
    ballData.captured = readLightGate();
    if (ballData.visible) {
        relBallCoords = Point(camera.ballAngle, camera.ballDist);
        absBallCoords = relBallCoords + botCoords;
        ballData.angle = camera.ballAngle;
        ballData.dist = camera.ballDist;
        ballData.x = absBallCoords.x;
        ballData.y = absBallCoords.y;

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
    }
}

bool readLightGate() { return analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH; }

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
    setMove(SPEED, robotAngle, 0);
}

void trackGoal() {
    float goalOffset;
    if (camera.oppAngle < 180)
        goalOffset = fmin(camera.oppAngle, 90);
    else
        goalOffset = max((360 - camera.oppAngle), -90);

    float goalMult = 1.5;
    robotAngle = camera.oppAngle + goalMult * goalOffset;
    setMove(SPEED, robotAngle, 0);
}

void guardGoal() {
    // align robot to x-coordinate of ball while tracking line
    // TODO: slowdown nearer to edges
    if (lineData.onLine) {
        if (abs(ballData.x) < GOALIE_LEEWAY_DIST) {
            // stop once ball is within certain horizontal distance
            float moveSpeed = 0;
        } else {
            float moveSpeed = max(goaliePID.update(abs(ballData.x)), MIN_SPEED);
        }

        float moveAngle = (ballData.angle > 180) ? -90 : 90;
        lineTrack = true;
    } else {
        Point target = Point(ballData.x, GOALIE_HOME_Y);
        goTo(target);
    }
}

void angleCorrect() { 
    moveData.rotation.val = cmpPID.update(cmp.readQuat()); 
}

#endif