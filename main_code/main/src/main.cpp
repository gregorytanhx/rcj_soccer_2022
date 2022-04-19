#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

PID goalieBallPID(3.5, 0.01, 2);
PID goalieGoalPID(2.5, 0, 1);

void normal() {
    // regular program
    updateAllData();
    goalieAttack = true;
    lineAvoid = true;
    lineTrack = false;
    if (currentRole() == Role::attack || goalieCharge) {
        if (currentRole() == Role::attack) {
            Serial.println("ATTACK MODE");
        } else if (goalieCharge) {
            Serial.println("GOALIE ATTACK MODE");
        }
        if (ballData.captured) {
            Serial.println("ball captured!");
        }
        Serial.print("Goalie Charge Timer: ");
        Serial.println(millis() - goalieChargeTimer);
        if (millis() - goalieChargeTimer > 5000) {
            // stop goalie from charging after a while
            goalieCharge = false;
            previouslyCharging = true;
        }
        // attack program
        if (ballData.captured) {
            Serial.println("Tracking Goal");
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
            Serial.println("Tracking Ball");
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
        if (goalieCharge) moveData.speed.val = 50;

        // updateKick();
        updateDribbler();
        angleCorrect();
        sendLayer1();

    } else {
        // defence program
        lineTrack = true;
        lineAvoid = false;
        printLightData();
        // printLightData();

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

        if (!lineData.onLine) {
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

                robotAngle =
                    fmod(camera.ownGoalAngle + goalOffset * goalMult, 360);

                moveSpeed = 50;
            } else if (camera.ownGoalDist < 29) {
                // robot is between goal and penalty area, move forward
                robotAngle = 0;
                moveSpeed = 50;
            }
        } else if (ballData.visible && ballData.dist < 80 &&
                   abs(camera.ownGoalAngle - 180) < 39) {
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
            Serial.print("Ball Angle: ");
            Serial.print(ballData.angle);
            Serial.print(" Error: ");
            Serial.println(error);
            if (error < 5) moveSpeed = 0;

        } else {
            Serial.println("Returning to goal centre");
            // if ball not visible, align to goal centre
            // since robot is line tracking, simply set target angle to own goal
            // angle
            Serial.println(camera.ownGoalAngle);
            if (camera.ownGoalAngle < 180)
                robotAngle = 90;
            else
                robotAngle = 270;
            // target goal angle is 180
            float error = abs(camera.ownGoalAngle - 180);

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
        bool ballScored = false;
        bool botPosFound = false;
        bool ballPosFound = false;
        bool posFound = false;
        int side = 0;  // 0 for side away from wall, 1 for side next to wall
        // always face yellow
        while (((!firstTime && millis() - timer < 1100) || !ballPosFound ||
                !posFound) &&
               !challenge3) {
            updateAllData();
            if (!posFound) {
                if (bbox.tofVals[1] < 400 && bbox.tofVals[3] > 600) {
                    side = 0;
                } else if (bbox.tofVals[3] < 400 && bbox.tofVals[1] > 600) {
                    side = 1;
                }
                if (ballData.visible && !ballPosFound) {
                    if ((ballData.angle > 50 && ballData.angle < 130) || ballData.angle ) {
                        // ball is directly in front of robot,
                        challenge has moved
                        // on to next stage
                        ballPosFound = true;
                        challenge3 = true;
                }
        }
    }

    // while (1) {
    //     // face yellow goal by default
    //     bool ballScored = false;
    //     bool botPosFound = false;
    //     bool ballPosFound = false;
    //     bool posFound = false;
    //     int side = 0;

    //     // treat yellow as front of field
    //     camera.side = Side::facingYellow;

    //     while (((!firstTime && millis() - timer < 1100) || !ballPosFound ||
    //             !posFound) &&
    //            !challenge3) {
    //         // if tof values are not confident, wait for other robot to stop
    //         // blocking
    //         updateAllData();
    //         if (camera.blueVisible && camera.yellowVisible && !posFound) {
    //             if (camera.blueAngle > 270 && camera.yellowAngle < 90) {
    //                 side = 0;
    //                 posFound = true;
    //             } else if (camera.yellowAngle > 270 && camera.blueAngle < 90)
    //             {
    //                 side = 1;
    //                 posFound = true;
    //             }
    //         }

    //         if (ballData.visible && !ballPosFound) {
    //             if (ballData.angle < 10 || ballData.angle > 350) {
    //                 // ball is directly in front of robot, challenge has
    //                 moved
    //                 // on to next stage
    //                 ballPosFound = true;
    //                 challenge3 = true;

    //                 heading = 90;
    //                 while (abs(camera.frontVector.getAngle() - heading) > 5)
    //                 {
    //                     updateAllData();
    //                     angleCorrect(heading);
    //                     moveData.angSpeed.val = 100;
    //                     sendLayer1();
    //                 }
    //             } else if (camera.blueVisible &&
    //                        abs(camera.blueAngle - ballData.angle) < 90) {
    //                 // ball close to blue goal
    //                 ballPosFound = true;
    //                 scoringBlue = true;
    //             } else if (camera.yellowVisible &&
    //                        abs(camera.yellowAngle - ballData.angle) < 90) {
    //                 ballPosFound = true;
    //                 scoringBlue = false;
    //             }
    //         }
    //         setMove(0, 0, 0);
    //         sendLayer1();
    //     }

    //     // use camera based correction cus compass confirmed fucked after
    //     // spinning so many times
    //     if (!challenge3) {
    //         lineTrack = true;
    //         if ((scoringBlue && side == 0) || (!scoringBlue && side == 1)) {
    //             while (bbox.tofVals[1] > 420 && bbox.tofVals[3] < 1800) {
    //                 updateAllData();
    //                 setMove(70, 270, 0);
    //                 angleCorrect(heading);
    //                 sendLayer1();
    //             }
    //         } else {
    //             while (bbox.tofVals[3] > 420 && bbox.tofVals[1] < 1800) {
    //                 updateAllData();
    //                 setMove(70, 90, 0);
    //                 angleCorrect(heading);
    //                 sendLayer1();
    //             }
    //         }
    //         lineTrack = false;
    //         while (!ballScored) {
    //             updateAllData();
    //             if (lastKickTime && millis() - lastKickTime > 500)
    //                 ballScored = true;
    //             if (ballData.captured) {
    //                 // turn off front dribbler + turn on kicker if facing
    //                 // front goal and 40cm away
    //                 int target = 0;
    //                 if (scoringBlue) target = 180;
    //                 // spin to face goal
    //                 while (abs(camera.frontVector.getAngle() - target) > 5) {
    //                     updateAllData();
    //                     angleCorrect(target);
    //                     moveData.angSpeed.val = 100;
    //                     sendLayer1();
    //                 }
    //                 kick = true;
    //                 dribble = false;
    //                 lastKickTime = millis();
    //                 angleCorrect(target);
    //             } else if (ballData.visible) {
    //                 trackBall();
    //                 // turn on front dribbler if ball within +-50 deg of
    //                 // front AND closer than 50 cm
    //                 if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
    //                     camera.ballDist < 50) {
    //                     dribble = true;
    //                 }
    //             } else {
    //                 goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
    //             }
    //             updateKick();
    //             if (!ballData.captured) angleCorrect(heading);
    //             sendLayer1();
    //         }
    //         lineTrack = true;
    //         heading = (heading + 180) % 360;

    //         // reverse until line
    //         while (!lineData.onLine) {
    //             updateAllData();
    //             setMove(60, 180, 0);
    //             angleCorrect(heading);
    //             sendLayer1();
    //         }
    //         // go back to field centre
    //         if (scoringBlue && side == 0 || !scoringBlue && side == 1) {
    //             while (bbox.tofVals[1] > 1200 && bbox.tofVals[3] < 1100) {
    //                 updateAllData();
    //                 setMove(70, 270, 0);
    //                 angleCorrect(heading);
    //                 sendLayer1();
    //             }
    //         } else {
    //             while (bbox.tofVals[3] > 1200 && bbox.tofVals[1] < 1100) {
    //                 updateAllData();
    //                 setMove(70, 90, 0);
    //                 angleCorrect(heading);
    //                 sendLayer1();
    //             }
    //         }
    //     } else {
    //         // move on to challenge 3
    //         heading = 90;

    //         if (millis() - goalieChargeTimer > 3000) {
    //             // stop goalie from charging after 3s
    //             goalieCharge = false;
    //         }
    //         // attack program
    //         if (ballData.captured) {
    //             trackGoal();
    //             // turn off front dribbler + turn on kicker if facing front
    //             // goal and 50cm away from goal
    //             if (camera.oppGoalDist < 50 &&
    //                 (camera.oppGoalAngle < 30 || camera.oppGoalAngle > 330)
    //                 && (robotAngle < 60 || robotAngle > 300)) {
    //                 // kick every 1.5s
    //                 if (millis() - lastKickTime > 1500) {
    //                     dribble = false;
    //                     kick = true;
    //                     lastKickTime = millis();
    //                     lastDribbleTime = millis();
    //                 } else {
    //                     kick = false;
    //                 }
    //             }
    //             if (millis() - lastDribbleTime > 500) {
    //                 // if kick failed, turn dribbler back on
    //                 dribble = true;
    //             }
    //         } else if (ballData.visible) {
    //             trackBall();
    //             // turn on front dribbler if ball within +-50 deg of front
    //             // AND closer than 50 cm
    //             if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
    //                 camera.ballDist < 50) {
    //                 dribble = true;
    //             }
    //         } else {
    //             goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
    //         }
    //         // move at faster speed if goalie is charging
    //         if (goalieCharge) moveData.speed.val = 80;

    //         updateKick();
    //         updateDribbler();
    //         angleCorrect();
    //         sendLayer1();
    //     }
    //     timer = millis();
    // }
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
    int distThresh;
    while (1) {
        updateAllData();
        // camera.printData();
        //  stop at all neutral points except middle
        if (pts[cnt] == CentreDot) {
            distThresh = 200;
        } else
            distThresh = 90;

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
        cnt %= 6;
    }
}

void turnToAng(int ang) {
    while (heading != ang) {
        Serial.println(heading);
        updateAllData();
        setMove(0, 0, (heading - ang) * 0.2, 30);
        sendLayer1();
    }
}

void setup() {
#ifdef SET_ID
    eeprom_buffered_write_byte(EEPROM_ID_ADDR, ID)
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
        // defaultRole = robotID == 0 ? Role::attack : Role::defend;
        defaultRole = Role ::defend;
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    Serial.begin(9600);
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    IMUSerial.begin(STM32_BAUD);
    camera.begin();
    bt.begin();
    bbox.begin();

    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWriteResolution(8);
    analogWriteFrequency(DRIBBLER_PIN, 1000);
    analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    delay(4000);
    lightGateVal.begin();
    // calibIMU();
    pinMode(LED_BUILTIN, OUTPUT);
    while (IMUSerial.available() == 0) {
        delay(1);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // float error = camera.yellowAngle
    //  setMove(0,0, )
    //  sendLayer1();
    robot1();
    // normal();

    // updateAllData();
    // //printLightData();
    // //camera.printData();
    // if (camera.ballVisible) {
    //     trackBall();
    //     if (ballData.captured) {
    //         kick = true;
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
    // angleCorrect();
    // sendLayer1();
}