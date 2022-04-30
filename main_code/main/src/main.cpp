#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

PID goalieBallPID(3.5, 0.01, 2);
PID goalieGoalPID(2.5, 0, 1);

void striker() {
    updateAllData();
    slowDown();
    bbox.printTOF();
    bool goalCorrect = true;
    if (ballData.captured && abs(nonReflex(ballData.angle)) < 50 &&
        ballData.dist < 30) {
        // // turn off front dribbler + turn on kicker if facing front goal and
        // // 50cm away
        // if (camera.oppGoalDist <= 45 &&
        //     (camera.oppGoalAngle < 30 || camera.oppGoalAngle > 330) &&
        //     (robotAngle < 60 || robotAngle > 300)) {
        //     // kick every 1.5s
        //     if (millis() - lastKickTime > 1500) {
        //         dribble = false;
        //         kick = true;
        //         lastKickTime = millis();
        //         lastDribbleTime = millis();
        //     }
        // }
        // if (millis() - lastDribbleTime > 3000) {
        //     // if kick failed, turn dribbler back on
        //     dribble = true;
        // }
        setMove(runningSpeed, 0, 0);

        if (camera.oppGoalDist < 55) {
            kick = true;
        } else {
            kick = false;
        }
        Serial.println("ball captured!");

    } else if (camera.ballVisible) {
        Serial.println("Tracking Ball");
        trackBall();
        if ((camera.ballAngle < 60 || camera.ballAngle > 300) &&
            camera.ballDist < 70) {
            dribble = true;
        } else {
            dribble = false;
        }
        kick = false;
    } else {
        goalCorrect = false;
        kick = false;
        dribble = false;
        goTo(neutralPoints[CentreDot], 90);
    }
    int distThresh = camera.side == Side::facingBlue ? 41 : 43;
    if (abs(nonReflex(camera.oppGoalAngle)) < 60 &&
        camera.oppGoalDist <= distThresh) {
        lineAvoid = false;
        Serial.println("returning to centre!");
        goTo(neutralPoints[CentreDot], 90);
    } else {
        lineAvoid = true;
    }

    // tof out detection
    if (bbox.outAngle > 0) {
        goalCorrect = false;
        Serial.print("OUT");
        Serial.print(" Angle: ");
        Serial.println(bbox.outAngle);
        setMove(runningSpeed, bbox.outAngle, 0);
    }
    if (camera.ownGoalDist < 50)
        lineAvoid = false;
    else
        lineAvoid = true;
    // updateLineControl();
    updateDribbler();
    updateKick();
    // if (goalCorrect && camera.oppGoalVisible) {
    //     aimGoal();
    // } else {
    //     angleCorrect();
    // }
    angleCorrect();

    sendLayer1();
}

void goalie() {
    updateAllData();
    // defence program
    lineTrack = true;
    lineAvoid = false;
    printLightData();
    // goalieAttack = true;

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

            moveSpeed = constrain(camera.ownGoalDist * 0.9, 30, 50);;
        } else if (camera.ownGoalDist < 29) {
            // robot is between goal and penalty area, move forward
            robotAngle = 0;
            moveSpeed = 50;
        }
    } else if (ballData.visible && ballData.dist < 80 &&
               camera.ownGoalAngle < 205 && camera.ownGoalAngle > 142) {
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

    } else {
        Serial.println("Returning to goal centre");
        // if ball not visible, align to goal centre
        // since robot is line tracking, simply set target angle to own goal
        // angle
        float centreAngle = 170;

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
    sendLayer1();
}

void normal() {
    // regular program
    goalieAttack = false;
    lineAvoid = true;
    lineTrack = false;
    if (currentRole() == Role::attack || goalieCharge) {
        if (currentRole() == Role::attack) {
            Serial.println("ATTACK MODE");
        } else if (goalieCharge) {
            Serial.println("GOALIE ATTACK MODE");
        }
        Serial.print("Goalie Charge Timer: ");
        Serial.println(millis() - goalieChargeTimer);
        if (millis() - goalieChargeTimer > 5000) {
            // stop goalie from charging after a while
            goalieCharge = false;
            previouslyCharging = true;
        }
        // attack program
        striker();

    } else {
        goalie();
    }
}

void moveInCircle() {
    MyTimer circleTimer(5);
    int dir = 0;

    while (true) {
        if (circleTimer.timeHasPassed()) {
            dir++;
            dir %= 360;
        }
        setMove(50, dir, 0);
        // angleCorrect();
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

void robot1() {
    // this shit damn cancer
    // robot 1 program for SG Open technical challenge
    // challenge 2: score ball into goal that it is closest to then go to other
    // spot
    // challenge 3: ball is randomly positioned on either side of field, score
    // ball into any goal

    bool firstTime = true;
    bool challenge3 = false;
    bool scoringBlue = false;
    long kickTime = 0;
    long timer = 0;
    int lr, fb;
    int side = 0;  // 0 for side away from wall, 1 for side next to wall

    while (1) {
        if (!challenge3) {
            bool botPosFound = false;
            bool ballPosFound = false;
            bool posFound = false;
            int leftCnt, rightCnt = 0;

            // always face yellow
            int c3cnt = 0;
            int blueCnt = 0;
            int yellowCnt = 0;
            while ((ballPosFound == false || posFound == false) &&
                   challenge3 == false) {
                updateAllData();
                camera.printData();
                if (!posFound) {
                    bbox.printTOF();
                    Serial.print(bbox.tofVals[1]);
                    Serial.print(" ");
                    Serial.println(bbox.tofVals[3]);
                    if (tof.vals[3] < 200 && tof.vals[1] > 600) {
                        leftCnt++;

                    } else if (tof.vals[1] < 200 && tof.vals[3] > 600) {
                        rightCnt++;
                    }
                    if (leftCnt >= 10) {
                        side = 1;
                        posFound = true;
                        Serial.println("SIDE NEXT TO WALL");
                    } else if (rightCnt >= 10) {
                        side = 0;
                        posFound = true;
                        Serial.println("SIDE AWAY FROM WALL");
                    }
                }
                Serial.print(ballPosFound);
                Serial.print(" ");
                Serial.println(posFound);

                if (ballData.visible && !ballPosFound) {
                    Serial.println("BALL POS NOT FOUND");
                    if (camera.blueVisible &&
                        abs(camera.blueAngle - ballData.angle) < 90) {
                        blueCnt++;

                    } else if (camera.yellowVisible &&
                               abs(camera.yellowAngle - ballData.angle) < 90) {
                        yellowCnt++;
                    } else {
                        c3cnt = 0;
                        blueCnt = 0;
                        yellowCnt = 0;
                    }
                    Serial.println(c3cnt);
                    if (yellowCnt >= 10) {
                        ballPosFound = true;
                        scoringBlue = false;
                        Serial.println("BALL CLOSER TO YELLOW");
                    } else if (blueCnt >= 10) {
                        ballPosFound = true;
                        scoringBlue = true;
                        Serial.println("BALL CLOSER TO BLUE");
                    }
                }
                if (!ballData.visible) {
                    c3cnt++;
                } else {
                    c3cnt = 0;
                }
                if (c3cnt >= 10) {
                    ballPosFound = true;
                    challenge3 = true;
                    Serial.println("CHALLENGE 3");
                }
            }
        }
    }

    firstTime = false;
    lineTrack = true;
    fb = scoringBlue ? 2 : 0;
    lr = side == 1 ? 3 : 1;

    if (!challenge3) {
        while (!TOFtoPoint(450, 170, fb, lr)) {
            Serial.println(scoringBlue);
            updateAllData();
            angleCorrect();
            sendLayer1();
        }

        setMove(0, 0, 0);
        sendLayer1();

        int sign = side == 1 ? 1 : -1;

        // side next to wall in lab
        turnToAng(90 * sign);
        Serial.print("TURNED");
        if (scoringBlue) {
            if (side == 1)
                lr = 1;
            else
                lr = 3;
        } else {
            if (side == 1)
                lr = 3;
            else
                lr = 1;
        }
        lineTrack = false;
        lineAvoid = false;
        while (!TOFtoPoint(500, 370, 2, lr)) {
            updateAllData();

            angleCorrect(90 * sign);
            sendLayer1();
        }
        lineTrack = true;
        setMove(0, 0, 0);
        sendLayer1();

        while (readLightGate() > 40) {
            Serial.println("moving to ball");
            updateAllData();
            setMove(30, ballData.angle, 0);
            angleCorrect(90 * sign);
            sendLayer1();
        }
        // flick ball into goal
        if (scoringBlue) {
            setMove(0, 0, 60 * sign, 20);
        } else {
            setMove(0, 0, -60 * sign, 20);
        }

        sendLayer1();
        delay(50);
        lineTrack = false;
        while (!TOFtoPoint(200, 370, 0, lr)) {
            updateAllData();
            angleCorrect(90 * sign);
            sendLayer1();
        }
        turnToAng(0);

        // move to next point and wait
        lineTrack = true;

        int tmp = side == 1 ? 1 : -1;

        while (!goTo(Point(tmp * 500, 0), 70)) {
            updateAllData();
            angleCorrect();
            sendLayer1();
        }
        timer = millis();
        while (millis() - timer < 1100) {
            updateAllData();
            angleCorrect();
            sendLayer1();
        }
    } else {
        bool scored = false;
        while (!scored) {
            striker();
            if (kick) scored = true;
        }

        int tmp = side == 1 ? 1 : -1;
        while (!goTo(Point(tmp * 500, 0), 70)) {
            updateAllData();
            angleCorrect();
            sendLayer1();
        }
        timer = millis();
        while (millis() - timer < 1100) {
            updateAllData();
            angleCorrect();
            sendLayer1();
        }
        // automatically swap sides
        if (side == 1)
            side = 0;
        else
            side = 1;
        // printLightData();
        // camera.printData();
    }
}

void robot2() {
    // robot 2 program for SG Open technical challenge
    // cycle between 5 neutral points

    // points pts[] = {BottomLeftDot,  TopLeftDot,  CentreDot,
    //                 BottomRightDot, TopRightDot, CentreDot};
    points pts[] = {TopLeftDot,    TopRightDot,
                    CentreDot, BottomLeftDot, BottomRightDot};
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
void setup() {
#ifdef SET_ID
    eeprom_buffered_write_byte(EEPROM_ID_ADDR, ID)
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
        defaultRole = robotID == 0 ? Role::defend : Role::defend;
    // defaultRole = Role ::defend;
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
    // analogWriteResolution(8);
    // pinMode(DRIBBLER_PIN, OUTPUT);

    // analogWriteFrequency(DRIBBLER_PIN, 1000);
    // analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    // time = millis();
    // calibIMU();
    pinMode(LED_BUILTIN, OUTPUT);
    // while (IMUSerial.available() == 0) {
    //     delay(1);
    // }
    // while (millis() - time < 3000) {
    //     delay(1);
    // }

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // float error = camera.yellowAngle
    //  setMove(0,0, )
    //  sendLayer1();
    // updateAllData();
    // Serial.print("Opp goal dist: ");
    // Serial.println(camera.oppGoalDist);

    // TODO: SPEED CONTROL BASE ON ROBOT MOVEMENT ANGLE
    //striker();

    // printLightData();
    // normal();
    //  updateAllData();
    // bbox.printTOF();

    // bbox.print();
    //robot2();
    updateAllData();
    // bt.printData();
    // readLayer1();
    //printLightData();
    // sendLayer1();
   
    // updateAllData();
    // printLightData();

   
    // normal();
    //  lineAvoid = false;
    // robot1();
    //  updateAllData();
    //  updateAllData();
    //  bbox.printTOF();

    // printIMU();
    // bbox.printTOF();
    // Serial.println(camera.oppGoalDist);

    // updateAllData();
    // printIMU();

    // striker();
    // updateAllData();
    // bbox.printTOF();
    // setMove(50,0,0);
    // sendLayer1();
    // normal();

    // Serial.println(readLightGate());

    // setMove(50,0,0);
    // sendLayer1();
    // striker();

    // lineTrack = true;
    // updateAllData();
    // goTo(Point(-500, 0));
    // angleCorrect();
    // sendLayer1();
    // Serial.print("TURNED");
    // while (lineData.onLine) {
    //     Serial.println("FUCk");
    //     updateAllData();
    //     setMove(50, 0, 0);
    //     angleCorrect(90);
    //     sendLayer1();
    // }
    // lineTrack = true;

    // while (readLightGate() > 50) {
    //     updateAllData();
    //     setMove(30, 0, 0);
    //     angleCorrect(90);
    //     sendLayer1();
    // }
    // setMove(0, 0, -60, 20);
    // sendLayer1();

    // normal();

    // updateAllData();
    // //printLightData();
    // //camera.printData();
    // if (camera.ballVisible) {
    //     trackBall();
    //     if (ballData.captured) {
    //         setMove(50,0,0);
    //         if (camera.oppGoalDist < 50) {
    //             kick = true;
    //         }
    //     } else {
    //         kick = false;
    //     }
    // } else {
    //     goTo(neutralPoints[CentreDot], 90);
    // }

    // if (bbox.outAngle > 0) {
    //     Serial.println("OUT");
    //     setMove(50, bbox.outAngle, 0);
    // }
    // //updateLineControl();
    // updateKick();
    // angleCorrect();
    // sendLayer1();
}