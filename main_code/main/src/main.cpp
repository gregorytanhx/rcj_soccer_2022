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
    lineTrack = false;
    if (currentRole() == Role::attack || goalieCharge) {
        if (currentRole() == Role::attack) {
            Serial.println("ATTACK MODE");
        } else if (goalieCharge) {
            Serial.println("GOALIE ATTACK MODE");
        }
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
            } else {
                dribble = false;
            }
        } else {
            dribble = false;
            goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
        }
        // move at faster speed if goalie is charging
        if (goalieCharge) moveData.speed.val = 80;

        updateKick();
        updateDribbler();
        angleCorrect();
        sendLayer1();

    } else {
        // defence program
        lineTrack = true;
        lineAvoid = false;
        Serial.println("DEFENCE MODE");
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

            robotAngle = fmod(camera.ownGoalAngle + goalOffset * goalMult, 360);
            moveSpeed = 70;

        } else if (ballData.visible && ballData.dist < 100) {
            // if ball is visible, align to ball
            // since robot is line tracking, simply set target angle to ball
            // angle
            robotAngle = ballData.angle;
            // target ball angle is zero
            // TODO: add compass angle to ball angle when that works
            float error = abs(ballData.angle);
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
    bool challenge3 = false;
    bool scoringBlue = false;
    long kickTime = 0;
    long timer = 0;
    int heading = 0;

    while (1) {
        // face yellow goal by default
        bool ballScored = false;
        bool botPosFound = false;
        bool ballPosFound = false;
        bool posFound = false;
        int side = 0;  // 0 for side away from wall, 1 for side next to wall

        // treat yellow as front of field
        camera.side = Side::facingYellow;

        while (((!firstTime && millis() - timer < 1100) || !ballPosFound ||
                !posFound) &&
               !challenge3) {
            // if tof values are not confident, wait for other robot to stop
            // blocking
            updateAllData();
            if (camera.blueVisible && camera.yellowVisible && !posFound) {
                if (camera.blueAngle > 270 && camera.yellowAngle < 90) {
                    side = 0;
                    posFound = true;
                } else if (camera.yellowAngle > 270 && camera.blueAngle < 90) {
                    side = 1;
                    posFound = true;
                }
            }

            if (ballData.visible && !ballPosFound) {
                if (ballData.angle < 10 || ballData.angle > 350) {
                    // ball is directly in front of robot, challenge has moved
                    // on to next stage
                    ballPosFound = true;
                    challenge3 = true;

                    heading = 90;
                    while (abs(camera.frontVector.getAngle() - heading) > 5) {
                        updateAllData();
                        angleCorrect(heading);
                        moveData.angSpeed.val = 100;
                        sendLayer1();
                    }
                } else if (camera.blueVisible &&
                           abs(camera.blueAngle - ballData.angle) < 90) {
                    // ball close to blue goal
                    ballPosFound = true;
                    scoringBlue = true;
                } else if (camera.yellowVisible &&
                           abs(camera.yellowAngle - ballData.angle) < 90) {
                    ballPosFound = true;
                    scoringBlue = false;
                }
            }
            setMove(0, 0, 0);
            sendLayer1();
        }

        // use camera based correction cus compass confirmed fucked after
        // spinning so many times
        if (!challenge3) {
            lineTrack = true;
            if ((scoringBlue && side == 0) || (!scoringBlue && side == 1)) {
                while (bbox.tofVals[1] > 420 && bbox.tofVals[3] < 1800) {
                    updateAllData();
                    setMove(70, 270, 0);
                    angleCorrect(heading);
                    sendLayer1();
                }
            } else {
                while (bbox.tofVals[3] > 420 && bbox.tofVals[1] < 1800) {
                    updateAllData();
                    setMove(70, 90, 0);
                    angleCorrect(heading);
                    sendLayer1();
                }
            }
            lineTrack = false;
            while (!ballScored) {
                updateAllData();
                if (lastKickTime && millis() - lastKickTime > 500)
                    ballScored = true;
                if (ballData.captured) {
                    // turn off front dribbler + turn on kicker if facing
                    // front goal and 40cm away
                    int target = 0;
                    if (scoringBlue) target = 180;
                    // spin to face goal
                    while (abs(camera.frontVector.getAngle() - target) > 5) {
                        updateAllData();
                        angleCorrect(target);
                        moveData.angSpeed.val = 100;
                        sendLayer1();
                    }
                    kick = true;
                    dribble = false;
                    lastKickTime = millis();
                    angleCorrect(target);
                } else if (ballData.visible) {
                    trackBall();
                    // turn on front dribbler if ball within +-50 deg of
                    // front AND closer than 50 cm
                    if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
                        camera.ballDist < 50) {
                        dribble = true;
                    }
                } else {
                    goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
                }
                updateKick();
                if (!ballData.captured)
                    angleCorrect(heading);
                sendLayer1();
            }
            lineTrack = true;
            heading = (heading + 180) % 360;

            // reverse until line
            while (!lineData.onLine) {
                updateAllData();
                setMove(60, 180, 0);
                angleCorrect(heading);
                sendLayer1();
            }
            // go back to field centre
            if (scoringBlue && side == 0 || !scoringBlue && side == 1) {
                while (bbox.tofVals[1] > 1200 && bbox.tofVals[3] < 1100) {
                    updateAllData();
                    setMove(70, 270, 0);
                    angleCorrect(heading);
                    sendLayer1();
                }
            } else {
                while (bbox.tofVals[3] > 1200 && bbox.tofVals[1] < 1100) {
                    updateAllData();
                    setMove(70, 90, 0);
                    angleCorrect(heading);
                    sendLayer1();
                }
            }
        } else {
            // move on to challenge 3
            heading = 90;

            if (millis() - goalieChargeTimer > 3000) {
                // stop goalie from charging after 3s
                goalieCharge = false;
            }
            // attack program
            if (ballData.captured) {
                trackGoal();
                // turn off front dribbler + turn on kicker if facing front
                // goal and 50cm away from goal
                if (camera.oppGoalDist < 50 &&
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
                // turn on front dribbler if ball within +-50 deg of front
                // AND closer than 50 cm
                if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
                    camera.ballDist < 50) {
                    dribble = true;
                }
            } else {
                goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
            }
            // move at faster speed if goalie is charging
            if (goalieCharge) moveData.speed.val = 80;

            updateKick();
            updateDribbler();
            angleCorrect();
            sendLayer1();
        }
        timer = millis();
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
        updateAllData();
        goTo(neutralPoints[pts[cnt]]);
        // stop at all neutral points except middle
        if (reachedPoint(neutralPoints[pts[cnt]], 30)) {
            lineTrack = false;
            if (pts[cnt] != CentreDot) {
                long timer = millis();
                while (millis() - timer < 1100) {
                    updateAllData();
                    setMove(0, 0, 0);
                    sendLayer1();
                }
            }
            cnt++;
        }
        camAngleCorrect();
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
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    Serial.begin(9600);
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    camera.begin();
    bt.begin();
    // cmp.begin();
    bbox.begin();

   
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWriteFrequency(DRIBBLER_PIN, 1000);
    analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    delay(DRIBBLER_WAIT);
    lightGateVal.begin();

    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    robot2();
    
    // camera.printData();
    // // 
    // if (camera.ballVisible) {
    //     trackBall();
    // } else {
    //     setMove(0,0,0);
    //     //goTo(neutralPoints[CentreDot]);
    // }
    // camAngleCorrect();
    // sendLayer1();
   
    // camera.printData();
    
    // camAngleCorrect();
    // //updateKick();
    // sendLayer1();
    // Serial.println(cmp.read());
    //  cmp.read();
    //  cmp.calibrate();
    //  TODO: Test TOF localisation
    // updateBluetooth();
    // if (bt.isConnected) {
    //     if (bt.newData) {
    //         Serial.print("Ball X: ");
    //         Serial.print(bt.otherData.ballData.x);
    //         Serial.print(" Ball Y: ");
    //         Serial.print(bt.otherData.ballData.y);
    //         Serial.println();
    //     }
        
    // } else {
    //     Serial.println("Disconnected");
    // }
    
    
   
    // front, left, back, right
    // if (readTOF()) {
    //     updatePosition();
    //     updateDebug();
    //     bbox.printTOF();
    //     // bbox.checkFieldDims();
    //     //  String dir[4] = {"Front", "Left", "Back", "Right"};
    //     //  for (int i = 0; i < 4; i++) {
    //     //      Serial.print(dir[i] + "Raw : ");
    //     //      Serial.print(tof.vals[i]);
    //     //      Serial.print("  ");
    //     //  }
    //     Serial.println();
    // }

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