#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

void striker() {
    slowDown();
    bool goalCorrect = false;
    if (movingSideways) {
        movingSideways = !goTo(sidewaysCoordinate, 100);
    } else if (ballData.captured) {
        if (usingDribbler) {
            if (camera.oppGoalVisible) {
                // camera.printData();
                Serial.println("TRACKING GOAL");
                Serial.print("Goal Angle: ");
                Serial.print(camera.oppGoalAngle);
                Serial.print(" Goal Dist: ");
                Serial.print(camera.oppGoalDist);
                Serial.println();
                // max speed when aiming for goal
                runningSpeed = 100;
                trackGoal();
                // turn off front dribbler + turn on kicker if facing front goal
                // and 50cm away
                if (camera.oppGoalDist <= 52 &&
                    (camera.oppGoalAngle < 30 || camera.oppGoalAngle > 330) &&
                    (robotAngle < 60 || robotAngle > 300)) {
                    // kick every 1.5s
                    if (millis() - lastKickTime > 1500) {
                        dribble = false;
                        kick = true;
                        lastKickTime = millis();
                        lastDribbleTime = millis();
                    }
                }
                if (millis() - lastDribbleTime > 3000) {
                    // if kick failed, turn dribbler back on
                    dribble = true;
                    dribblerOnTime = millis();
                }
            } else {
                goTo(neutralPoints[CentreDot], 90);
                Serial.println("CAN'T SEE GOAL, RETURNING TO CENTRE");
            }

        } else {
            setMove(runningSpeed, 0, 0);

            if (camera.oppGoalDist <= 50) {
                kick = true;
            } else {
                kick = false;
            }
        }

        // Serial.println("ball captured!");
    } else if (camera.ballVisible) {
        Serial.println("TRACKING BALL");
        trackBall();
        if ((camera.ballAngle < 60 || camera.ballAngle > 300) &&
            camera.ballDist < 60) {
            dribble = true;
            dribblerOnTime = millis();
        } else if (millis() - dribblerOnTime > 1000) {
            dribble = false;
        }
        kick = false;
    } else {
        goalCorrect = false;
        kick = false;
        if (millis() - dribblerOnTime > 1000) {
            dribble = false;
        }
        goTo(neutralPoints[CentreDot], 90);
        // setMove(0,0,0);
        Serial.println("CAN'T SEE BALL, RETURNING TO CENTRE");
    }

    int distThresh = 42;
    // camera.printData(0);
    // avoid fully entering penalty area

    avoidLine();
    updateDribbler();
    updateKick();
    if (usingDribbler) {
        angleCorrect();
    } else if (goalCorrect && camera.oppGoalVisible) {
        aimGoal();
    } else {
        angleCorrect();
    }
}

void goalie() {
    // defence program
    lineTrack = true;
    lineAvoid = false;
    int goalAngLeft, goalAngRight, centreAngle, distThresh;
    if (robotID == 0) {
        centreAngle = 165;
        if (camera.ownGoalAngle > centreAngle)
            distThresh = 30;
        else
            distThresh = 34;
    } else {
        centreAngle = 165;
        if (camera.ownGoalAngle > centreAngle)
            distThresh = 30;
        else
            distThresh = 33;
    }
    if (previouslyCharging) {
        lastChargeTime = millis();
        previouslyCharging = false;
    }

    float goalYDist =
        camera.ownGoalDist * cos(deg2rad(180 - camera.ownGoalAngle));
    float goalXDist =
        camera.ownGoalDist * sin(deg2rad(centreAngle - camera.ownGoalAngle));
    if (millis() - lastChargeTime > 10000) {
        if (camera.ballVisible && camera.newData &&
            abs(lastBallAngle - camera.ballAngle) > 5 &&
            abs(lastBallDist - camera.ballDist) > 20) {
            lastBallMoveTime = millis();
        }
        if (millis() - lastBallMoveTime > 3000 && goalieAttack) {
            Serial.println("Charging");
            // goalie charges to score if ball has not moved for some time
            goalieCharge = true;
            goalieChargeTimer = millis();
            lineTrack = false;
            // move off white line first
            while (lineData.onLine) {
                updateAllData();
                setMove(60, 0, 0);
                angleCorrect();
                sendLayer1();
            }
            moveSpeed = 0;
            robotAngle = 0;
        }
    }

    if (goalYDist < 40 && camera.ownGoalDist < 75 && ballData.visible &&
        ballData.dist < 80) {
        // Serial.println("Aliging to ball");
        // if ball is visible, align to ball
        // since robot is line tracking, simply set target angle to ball
        // angle
        float xCo, yCo;
        xCo = relBall.x / 10;
        yCo = distThresh - goalYDist;
        robotAngle = mod(polarAngle(yCo, xCo), 360);
        // target ball angle is zero

        // speed now proportional to ball distance as well, need to tune pid
        // fuck

        //
        float distError = (80 / (abs(relBall.y) / 10)) + abs(relBall.x) / 10;
        float error = abs(nonReflex(ballData.angle)) + distError;
        moveSpeed = goalieBallPID.update(error);

        // Serial.print("speed:  ");
        // Serial.print(moveSpeed);

        // Serial.print(" Ball Angle: ");
        // Serial.print(ballData.angle);
        // Serial.print(" Ball Distance: ");
        // Serial.println(ballData.dist);
        // Serial.print("Angle: ");
        // Serial.println(robotAngle);
        // if (abs(nonReflex(ballData.angle)) < 1) moveSpeed = 0;
        if (((goalXDist < -18) && ballData.angle < 180) ||
            ((goalXDist > 20) && ballData.angle > 180)) {
            // Serial.println("STOPPED");
            moveSpeed = 0;
        }
        // Serial.print(" Speed: ");
        Serial.println(moveSpeed);

        moveSpeed = min(moveSpeed, 90);

    } else {
        // Serial.println("Returning to goal centre");
        // if ball not visible, align to goal centre
        // since robot is line tracking, simply set target angle to own goal
        // angle

        // target goal angle is 180
        float xCo, yCo;
        xCo = goalXDist;
        yCo = distThresh - goalYDist;
        robotAngle = polarAngle(yCo, xCo);
        float error = abs(camera.ownGoalAngle - centreAngle) *
                      sqrt(xCo * xCo + yCo * yCo);

        // Serial.print("Own goal angle: ");
        // Serial.println(camera.ownGoalAngle);
        // Serial.print("Angle: ");
        // Serial.println(robotAngle);
        moveSpeed = min(goalieGoalPID.update(error), 70);

        // Serial.print("Speed: ");
        // Serial.println(moveSpeed);
    }

    lineTrack = false;
    lineAvoid = false;
    setMove(moveSpeed, robotAngle, 0);
    lastBallAngle = camera.ballAngle;
    lastBallDist = camera.ballDist;
    // Serial.print("Angle: ");
    // Serial.println(robotAngle);
    // Serial.print("Own goal ang: ");
    // Serial.print(camera.ownGoalAngle);
    // Serial.print(" Own goal X: ");
    // Serial.print(goalXDist);
    // Serial.print(" Own goal Y: ");
    // Serial.println(goalYDist);

    angleCorrect();
}

void normal() {
    // regular program

    goalieAttack = false;
    // lineAvoid = true;
    // lineTrack = false;
    updateAllData();
    // printLightData();
    if (currentRole() == Role::attack) {
        // attack program
        // Serial.println("ATTACK");
        striker();

    } else {
        // defence program
        // Serial.println("DEFENCE");
        goalie();
    }
    // lineAvoid = false;
    // Serial.print("SPEED: ");
    // Serial.print(moveData.speed.val);
    // Serial.print(" ANGLE: ");
    // Serial.print(moveData.angle.val);
    // Serial.println();
    sendLayer1();
}

void moveInCircle() {
    MyTimer circleTimer(5);
    int dir = 360;
    lineAvoid = false;
    while (true) {
        readIMU();
        if (circleTimer.timeHasPassed()) {
            dir++;
            // if (dir == 0) dir = 360;
            dir %= 360;
        }
        setMove(50, dir, 0);
        angleCorrect();
        sendLayer1();
    }
}

void robot2() {
    // robot 2 program for SG Open technical challenge
    // cycle between 5 neutral points

    // points pts[] = {BottomLeftDot,  TopLeftDot,  CentreDot,
    //                 BottomRightDot, TopRightDot, CentreDot};
    points pts[] = {TopLeftDot, TopRightDot, CentreDot, BottomLeftDot,
                    BottomRightDot};
    int cnt = 0;
    lineAvoid = false;
    lineTrack = false;
    int distThresh;
    while (1) {
        updateAllData();

        // camera.printData();
        //  stop at all neutral points except middle
        if (pts[cnt] == CentreDot) {
            distThresh = 130;
        } else {
            distThresh = 50;
        }

        if (goTo(neutralPoints[pts[cnt]], distThresh)) {
            lineTrack = false;
            if (pts[cnt] != CentreDot) {
                long timer = millis();
                while (millis() - timer < 1100) {
                    updateAllData();
                    setMove(0, 0, 0);
                    angleCorrect();
                    sendLayer1();
                }
            }
            cnt++;
        }
        angleCorrect();
        sendLayer1();
        cnt %= 5;
    }
}
long time;
// #define SET_ID
void setup() {
#ifdef SET_ID
    EEPROM.write(EEPROM_ID_ADDR, 0);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    // defaultRole = robotID == 0 ? Role::defend : Role::attack;
    roleSwitching = false;
    movingSideways = false;
    defaultRole = Role::defend;
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    Serial.begin(9600);
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    IMUSerial.begin(STM32_BAUD);
    camera.begin();
    // camera.side = Side::facingBlue;

    camera.side = Side::facingYellow;
    bt.begin();
    bbox.begin();

    lightGateVal.begin();

    usingDribbler = true;
    if (usingDribbler) {
        analogWriteResolution(8);
        pinMode(DRIBBLER_PIN, OUTPUT);

        analogWriteFrequency(DRIBBLER_PIN, 1000);
        analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
        time = millis();
    }

    // calibIMU();
    pinMode(LED_BUILTIN, OUTPUT);
    while (IMUSerial.available() == 0) {
        delay(1);
    }
    if (usingDribbler) {
        while (millis() - time < 3000) {
            delay(1);
        }
    }

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}
long tmp = 0;
void loop() {
    // TODO: ADD FAILURE STATE FOR TOF, USE COMPASS IF TOF DIED
    // bbox.printTOF();
    // printIMU();
//  updateAllData();
//     setMove(0,0,0);
//     angleCorrect();
//     sendLayer1();
   normal();
    // tmp = micros();
    // normal();
    // Serial.println(micros() - tmp);
    //   bbox.checkFieldDims();
    // runningSpeed = 50;
    // updateAllData();
    // printIMU();
    // trackGoal();
    // angleCorrect();
    // sendLayer1();

    // moveInCircle();
    // updateAllData();
    // lineAvoid = false;
    // setMove(0,0,0);
    // printIMU();
    // angleCorrect();
    // sendLayer1();

    // if (millis() - tmp > 5000) {
    //     tmp = millis();
    //     if (kick) kick = false;
    //     else kick = true;
    // }

    // Serial.print("LIGHT GATE: ");
    // Serial.println(readLightGate());
    // updateDribbler();
    // updateKick();
    // printLightData();
    //  dribble = true;
    //  updateDribbler();
    // // bbox.printTOF();

    // bt.printData();
    // readLayer1();
    // printLightData();
    // sendLayer1();
}