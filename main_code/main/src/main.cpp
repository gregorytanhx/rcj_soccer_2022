#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

PID goalieBallPID(1.9, 0, 0);
PID goalieGoalPID(0.6, 0, 0);

void normal() {
    // regular program
    updateAllData();
    lineAvoid = true;
    if (currentRole() == Role::attack || goalieCharge) {
        if (millis() - goalieChargeTimer > 3000) {
            // stop goalie from charging after 3s
            goalieCharge = false;
        }
        // attack program
        if (ballData.captured) {
            trackGoal();
            // turn off front dribbler + turn on kicker if facing front goal and
            // 50cm away
            if (camera.oppGoalDist <= 50 &&
                (camera.oppGoalAngle < 30 || camera.oppGoalAngle > 330) &&
                (robotAngle < 60 || robotAngle > 300)) {
                // kick every 1.5s
                if (millis() - lastKickTime > 1500) {
                    dribble = false;
                    kick = true;
                    lastKickTime = millis();
                    lastDribbleTime = millis();
                } else {
                    kick = false;
                }
            }
            if (millis() - lastDribbleTime > 500) {
                // if kick failed, turn dribbler back on
                dribble = true;
            }
        } else if (ballData.visible) {
            trackBall();
            // turn on front dribbler if ball within +-50 deg of front AND
            // closer than 50 cm
            if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
                camera.ballDist < 50) {
                dribble = true;
            }
        } else {
            goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
        }
        // move at faster speed if goalie is charging
        if (goalieCharge) moveData.speed = 80;

        updateKick();
        updateDribbler();
        angleCorrect();
        sendLayer1();

    } else {
        // defence program
        lineTrack = true;
        if (camera.ballVisible && camera.newData &&
            abs(lastBallAngle - camera.ballAngle) < 1 &&
            abs(lastBallDist - camera.ballDist) < 20) {
            ballCnt++;
        } else {
            ballCnt = 0;
        }
        if (ballCnt >= 10) {
            // goalie charges to score if ball has not moved for some time
            goalieCharge = true;
            goalieChargeTimer = millis();
            lineTrack = false;
            // avoid moving in the wrong direction before swapping to attack
            // mode
            moveSpeed = 0;
            robotAngle = 0;
        }
        if (!lineData.onLine || camera.oppGoalDist > 75) {
            // return to goal area if not on line or too far from goal centre
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

            robotAngle = mod(camera.ownGoalAngle + goalOffset * goalMult, 360);
            moveSpeed = 70;

        } else if (ballData.visible && ballData.dist < 100) {
            // if ball is visible, align to ball
            // since robot is line tracking, simply set target angle to ball
            // angle
            robotAngle = ballData.ballAngle;
            // target ball angle is zero
            // TODO: add compass angle to ball angle when that works
            float error = abs(allData.angle);
            moveSpeed = goalieBallPID.update(error);
            moveSpeed = min(110, moveSpeed);

        } else {
            // if ball not visible, align to goal centre
            // since robot is line tracking, simply set target angle to own goal
            // angle
            robotAngle = camera.ownGoalAngle;
            // target goal angle is 180
            float error = abs(nonReflex(camera.ownGoalAngle - 180));
            moveSpeed = goalieBallPID.update(error);
            moveSpeed = min(110, moveSpeed);
        }

        setMove(moveSpeed, robotAngle, 0);
        lastBallAngle = camera.ballAngle;
        lastBallDist = camera.ballDist;
    }

    angleCorrect();
    sendLayer1();
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
        angleCorrect();
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

    // gay ass strategy for challenge 2: 
    // start robot perpendicular to line facing field
    // robot moves to top part corner of field (left TOF < 42 cm)
    // robot tracks ball like normal
    // once ball is caught, turn and kick into goal
    // turn 180 degrees
    // reverse until line is detected
    // move to centre of field 

    bool firstTime = true;

    while (1) {
        // face yellow goal by default
        points startPoint, endPoint;
        bool facingYellow = true;
        bool ballScored = false;
        bool botPosFound = false;
        bool ballPosFound = false;

        while ((!firstTime && millis() - timer < 1100) || !posFound ||
               !ballPosFound) {
            // if tof values are not confident, wait for other robot to stop
            // blocking

            updateAllData();
            if (tof.vals[1] < 350 && tof.vals[3] > 600) {
                // left tof closer to wall
                if ((camera.yellowAngle < 60 || camera.yellowAngle > 300) ||
                    (camera.blueAngle > 120 || camera.blueAngle < 240)) {
                    // starting at P1A
                    startPoint = LeftSide;
                    endPoint = RightSide;
                    posFound = true;
                } else if ((camera.blueAngle < 60 || camera.blueAngle > 300) ||
                           (camera.yellowAngle > 120 ||
                            camera.yellowAngle < 240)) {
                    // starting at P1B
                    startPoint = RightSide;
                    endPoint = LeftSide;
                    posFound = true;
                }

            } else if (tof.vals[1] > 600 && tof.vals[3] < 350) {
                // right tof closer to wall

                if ((camera.yellowAngle < 60 || camera.yellowAngle > 300) ||
                    (camera.blueAngle > 120 || camera.blueAngle < 240)) {
                    // starting at P1B
                    startPoint = RightSide;
                    endPoint = LefttSide;
                    posFound = true;
                } else if ((camera.blueAngle < 60 || camera.blueAngle > 300) ||
                           (camera.yellowAngle > 120 ||
                            camera.yellowAngle < 240)) {
                    // starting at P1A
                    startPoint = LeftSide;
                    endPoint = RightSide;
                    posFound = true;
                }
            }

            if (ballData.visible && !ballPosFound) {
                ballPosFound = true;
                if (ballData.ballAngle < 50 || ballData.ballAngle > 310) {
                    // ball closer to YELLOW goal
                    facingYellow = true;

                } else if (ballData.ballAngle < 130 ||
                           ballData.ballAngle > 230) {
                    // ball closer to BLUE goal
                    facingYellow = false;
                } else {
                    // challenge has entered third stage
                    // score yellow goal by default
                    facingYellow = true;
                }
            }
            setMove(0, 0, 0);
            sendLayer1();
        }

        // use camera based correction cus compass confirmed fucked after
        // spinning so many times

        long shotTime = 0;
        // run standard attack program, assume ball has been scored once kicked
        while (!ballScored) {
            updateAllData();
            if (millis() - shotTime < 500) {
                ballScored = true;
            }
            if (ballData.captured) {
                // turn off front dribbler + turn on kicker if facing front goal
                // and 40cm away
                float goalDist, goalAngle;
                if (facingYellow) {
                    goalDist = camera.yellowDist;
                    goalAngle = camera.yellowAngle;
                } else {
                    goalDist = camera.blueDist;
                    goalAngle = camera.blueAngle;
                }

                trackGoal(goalAngle);
                if (goalDist <= 400 && (goalAngle < 30 || goalAngle > 330) &&
                    (robotAngle < 60 || robotAngle > 300)) {
                    // kick every 1.5s
                    dribble = false;
                    kick = true;
                    shotTime = millis();
                }
                if (millis() - lastDribbleTime > 500) {
                    // if kick failed, turn dribbler back on
                    dribble = true;
                }
            } else if (ballData.visible) {
                trackBall();
                // turn on front dribbler if ball within +-50 deg of front AND
                // closer than 50 cm
                if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
                    camera.ballDist < 50) {
                    dribble = true;
                }
            } else {
                goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
            }

            updateKick();
            updateDribbler();

            // face the correct goal
            if (facingYellow)
                camAngleCorrect();
            else
                camAngleCorrect(180);

            sendLayer1();
        }

        // go to end point once ball has been scored
        while (!reachedPoint(neutralPoints[endPoint], 100)) {
            updateAllData();
            goTo(neutralPoints[endPoint]);
            if (facingYellow)
                camAngleCorrect();
            else
                camAngleCorrect(180);
            sendLayer1();
        }
    }
}

void robot2() {
    // robot 2 program for SG Open technical challenge
    // cycle between 5 neutral points

    points pts[5] = {TopLeftDot, TopRightDot, CentreDot, BottomLeftDot,
                     BottomRightDot};
    int cnt = 0;
    lineAvoid = false;
    while (1) {
        updateAll();
        goTo(neutralPoints[pts[cnt]]);
        // stop at all neutral points except middle
        if (reachedPoint(neutralPoints[pts[cnt]]) &&
            neutralPoints[pts[cnt]] != CentreDot) {
            lineTrack = false;
            long timer = millis();
            while (millis() - timer < 1100) {
                updateAll();
                setMove(0, 0, 0);
                sendLayer1();
            }
            cnt++;
        }
        angleCorrect();
        sendLayer1();
        cnt %= 5;
    }
}

void setup() {
#ifdef SET_ID
    eeprom_buffered_write_byte(EEPROM_ID_ADDR, ID)
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
        defaultRole = robotID == 0 ? Role::attack : Role::defend;

    Serial.begin(9600);
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    camera.begin();
    bt.begin();
    // cmp.begin();
    bbox.begin();

    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWriteFrequency(DRIBBLER_PIN, 1000);
    analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    delay(DRIBBLER_WAIT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // Serial.println(cmp.read());
    //  cmp.read();
    //  cmp.calibrate();
    //  TODO: Test TOF localisation

    // front, left, back, right
    if (readTOF()) {
        updatePosition();
        updateDebug();
        bbox.printTOF();
        // bbox.checkFieldDims();
        //  String dir[4] = {"Front", "Left", "Back", "Right"};
        //  for (int i = 0; i < 4; i++) {
        //      Serial.print(dir[i] + "Raw : ");
        //      Serial.print(tof.vals[i]);
        //      Serial.print("  ");
        //  }
        Serial.println();
    }

    // goTo(neutralPoints[CentreDot]);
    //  angleCorrect();
    //  sendLayer1();
    //  bbox.print();

    // // controlDribbler();
    // cmp.printCalib();
    // Serial.print("Euler: ");
    // Serial.println(cmp.readEuler());
    // Serial.print("Quaternion: ");
    // Serial.println(cmp.readQuat());

    // readLayer1();
    // if (lineData.onLine) {
    //     Serial.println("Line Detected");
    //     Serial.print("Angle: ");
    //     Serial.print(lineData.lineAngle.val);
    //     Serial.print(" Chord Length: ");
    //     Serial.println(lineData.chordLength.val);
    // }

    // camera.update();

    // heading = cmp.read();
    // readLayer4();
    // updatePosition();

    // if (bluetoothTimer.timeHasPassed()) {
    //     updateBluetooth();
    // }

    // updateBallData();
}