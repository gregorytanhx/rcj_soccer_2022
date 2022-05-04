#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

PID goalieBallPID(3.5, 0.01, 2);
PID goalieGoalPID(2.5, 0, 1);

void striker() {

    slowDown();
    bool goalCorrect = false;
    lineAvoid = false;
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
                trackGoal();
                // turn off front dribbler + turn on kicker if facing front goal
                // and 50cm away
                if (camera.oppGoalDist <= 52 &&
                    (camera.oppGoalAngle < 25 || camera.oppGoalAngle > 335) &&
                    (robotAngle < 40 || robotAngle > 320)) {
                    
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
                Serial.println("RETURNING TO CENTRE");
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
        Serial.println("Tracking Ball");
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
        // goTo(neutralPoints[CentreDot], 90);
        setMove(0,0,0);
        Serial.println("RETURNING TO CENTRE");
    }

    int distThresh = 42;
    // camera.printData(0);
    // avoid fully entering penalty area
    // if (abs(nonReflex(camera.oppGoalAngle)) < 30 &&
    //     camera.oppGoalDist <= distThresh) {
    //     lineAvoid = false;
    //     camera.printData(0);
    //     Serial.println(camera.oppGoalAngle);
    //     Serial.println(camera.oppGoalDist);
    //     Serial.println("returning to centre!");
    //     goTo(neutralPoints[CentreDot], 90);
    // } else {
    //     lineAvoid = true;
    // }

    // tof out detection
    // avoidLine();
    // if (camera.oppGoalDist < 50 || camera.ownGoalDist < 50) {
    //     lineAvoid = false;
    // }

    // if (camera.ownGoalDist < 50)
    //     lineAvoid = false;
    // else
    //     lineAvoid = true;
    // updateLineControl();
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
    int goalAngLeft, goalAngRight, centreAngle, tofThresh;
    if (robotID == 0) {
        goalAngRight = 195;
        goalAngLeft = 140;
        centreAngle = 165;
        tofThresh = 710;
    } else {
        goalAngRight = 205;
        goalAngLeft = 140;
        centreAngle = 180;
        tofThresh = 710;
    }
    if (previouslyCharging) {
        lastChargeTime = millis();
        previouslyCharging = false;
    }
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

    if (!lineData.onLine || camera.ownGoalDist > 75) {
        Serial.print("OWN GOAL DIST: ");
        Serial.println(camera.ownGoalDist);
        if (camera.ownGoalDist > 35) {
            Serial.println("Returning to goal");
            //  return to goal area if not on line or too far from goal
            //  centre
            float goalMult, goalOffset;
            int tempAng = camera.ownGoalAngle - 180;
            if (tempAng >= 0) {
                goalOffset = fmin(tempAng * 1.0, 90);
            } else {
                goalOffset = fmax(tempAng * 1.0, -90);
            }
            // add offset if goal angle is very steep
            if (camera.ownGoalAngle < 100 || camera.ownGoalAngle > 260) {
                goalMult = 0.3;
            } else {
                goalMult = 0;
            }

            robotAngle = fmod(camera.ownGoalAngle + goalOffset * goalMult, 360);

            moveSpeed = constrain(camera.ownGoalDist * 0.9, 30, 50);
            ;
        } else if (camera.ownGoalDist < 29) {
            // robot is between goal and penalty area, move forward
            robotAngle = 0;
            moveSpeed = 50;
        }
    } else if (ballData.visible && ballData.dist < 80) {
        Serial.println("Aliging to ball");
        // if ball is visible, align to ball
        // since robot is line tracking, simply set target angle to ball
        // angle
        if (ballData.angle < 180)
            robotAngle = 90;
        else
            robotAngle = 270;
        // target ball angle is zero
        // TODO: add compass angle to ball angle when that works
        float error = abs(nonReflex(ballData.angle));
        moveSpeed = goalieBallPID.update(error);

        moveSpeed = constrain(moveSpeed, 30, 70);
        Serial.print("Own goal angle: ");
        Serial.println(camera.ownGoalAngle);
        Serial.print("Ball Angle: ");
        Serial.print(ballData.angle);
        Serial.print(" Error: ");
        Serial.println(error);
        if (error < 1) moveSpeed = 0;
        if ((camera.ownGoalAngle > goalAngRight ||
             bbox.tofVals[1] < tofThresh) &&
                robotAngle == 90 ||
            (camera.ownGoalAngle < goalAngLeft ||
             bbox.tofVals[3] < tofThresh) &&
                robotAngle == 270) {
            moveSpeed = 0;
        }

    } else {
        Serial.println("Returning to goal centre");
        // if ball not visible, align to goal centre
        // since robot is line tracking, simply set target angle to own goal
        // angle

        if (camera.ownGoalAngle < centreAngle)
            robotAngle = 90;
        else
            robotAngle = 270;
        // target goal angle is 180
        float error = abs(camera.ownGoalAngle - centreAngle);
        Serial.print("Own goal angle: ");
        Serial.println(camera.ownGoalAngle);
        moveSpeed = goalieGoalPID.update(error);
        moveSpeed = constrain(moveSpeed, 30, 50);
        if (error < 3) moveSpeed = 0;
        Serial.print("Speed: ");
        Serial.println(moveSpeed);
    }
    // Serial.print("Angle: ");
    // Serial.println(robotAngle);
    setMove(moveSpeed, robotAngle, 0);
    lastBallAngle = camera.ballAngle;
    lastBallDist = camera.ballDist;
    Serial.print("Angle: ");
    Serial.println(robotAngle);

    angleCorrect();
   
}

void normal() {
    // regular program

    goalieAttack = false;
    lineAvoid = true;
    lineTrack = false;
    updateAllData();
    printIMU();
    if (currentRole() == Role::attack) {
        // attack program
        // Serial.println("ATTACK");
        striker();

    } else {
        // defence program
        // Serial.println("DEFENCE");
        goalie();
    }
    lineAvoid = false;
    Serial.print("SPEED: ");
    Serial.print(moveData.speed.val);
    Serial.print(" ANGLE: ");
    Serial.print(moveData.angle.val);
    Serial.println();
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

void turnToAng(int angle) {
    int cnt = 0;
    int polarity = angle / abs(angle);
    if (polarity == 0) polarity = 1;
    while (cnt < 10) {
        Serial.println(angle);
        updateAllData();
        float correction = (angle - heading) * 0.5;
        correction = constrain(correction, 7, 10);
        Serial.println(polarity * correction);
        setMove(0, 0, correction * polarity, 20);
        // if (heading < angle) {
        //     // clockwise
        //     Serial.println("here");
        //     setMove(0, 0, correction * polarity, 20);
        // } else if (heading > angle) {
        //     setMove(0, 0, -correction * polarity, 20);
        // } else {
        //     setMove(0, 0, 0);
        // }
        if (abs(heading - angle) < 2) {
            cnt++;
        } else {
            cnt = 0;
        }

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
//#define SET_ID
void setup() {
#ifdef SET_ID
    EEPROM.write(EEPROM_ID_ADDR, 0);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    // defaultRole = robotID == 0 ? Role::defend : Role::attack;
    roleSwitching = false;
    movingSideways = false;
    defaultRole = Role::attack;
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
    
    //   normal();
    runningSpeed = 50;
    updateAllData();
    printIMU();
    trackGoal();
    angleCorrect();
    sendLayer1();

    // moveInCircle();
     // updateAllData();
     // lineAvoid = false;
     // setMove(0,0,0);
     // printIMU();
     // angleCorrect();
     // sendLayer1();

     // dribble = true;
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