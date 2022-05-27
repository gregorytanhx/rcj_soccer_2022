#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

void striker() {
    bool goalCorrect = false;
    lineTrack = false;
    if (ballData.captured) {
        if (usingDribbler) {
            if (camera.oppGoalVisible) {
                attackState = chasingGoal;
                // camera.printData();
                // Serial.println("TRACKING GOAL");
                // Serial.print("Goal Angle: ");
                // Serial.print(camera.oppGoalAngle);
                // Serial.print(" Goal Dist: ");
                // Serial.print(camera.oppGoalDist);
                // Serial.println();
                // max speed when aiming for goal
                // runningSpeed = 80;
                trackGoal();
                // turn off front dribbler + turn on kicker if facing front goal
                // and 50cm away
                if (camera.oppGoalDist <= 50 &&
                    (camera.oppGoalAngle <= 20 || camera.oppGoalAngle >= 340) &&
                    (robotAngle < 45 || robotAngle > 315)) {
            
                    // kick every 1.5s
                    if (millis() - lastKickTime > 1500) {
                        dribble = false;
                        kick = true;
                        lastKickTime = millis();
                        lastDribbleTime = millis();
                    }
                } else {
                    kick = false;
                }

                if (millis() - lastDribbleTime > 3000) {
                    // if kick failed, turn dribbler back on
                    dribble = true;
                    dribblerOnTime = millis();
                }
            } else {
                attackState = returningToCentre;
                goTo(neutralPoints[CentreDot], 70);
                Serial.println("CAN'T SEE GOAL, RETURNING TO CENTRE");
            }

        } else {
            robotAngle = 0;
            attackState = chasingGoal;
            if (camera.oppGoalDist <= 50) {
                kick = true;
            } else {
                kick = false;
            }
        }

        // Serial.println("ball captured!");
    } else if (camera.ballVisible) {
        // Serial.println("TRACKING BALL");
        trackBall();
        attackState = chasingBall;
        if ((camera.ballAngle < 50 || camera.ballAngle > 310) &&
            camera.ballDist < 50) {
            dribble = true;
            dribblerOnTime = millis();
        } else if (millis() - dribblerOnTime > 1000) {
            dribble = false;
        }
        kick = false;
    } else {
        attackState = returningToCentre;
        goalCorrect = false;
        kick = false;
        if (millis() - dribblerOnTime > 1000) {
            dribble = false;
        }
        goTo(neutralPoints[CentreDot], 70);
        // setMove(0,0,0);
        // Serial.println("CAN'T SEE BALL, RETURNING TO CENTRE");
    }

    // int distThresh = 41;

    // if (camera.oppGoalDist < distThresh) {
    //     goTo(neutralPoints[CentreDot], 70);
    // }
    // avoid fully entering penalty area

    setMove(runningSpeed, robotAngle, 0);
    slowDown();

    if (prevRole == Role::defend) {
        // just switched over from defence, exit penalty area first
        inPenalty = true;
    }
    if (inPenalty) {
        lineAvoid = false;
        if (!lineData.onLine) inPenalty = false;
    }
    // else if (camera.oppGoalDist <= 41) {
    //     lineAvoid = false;
    //     if (!ballData.captured) {

    //         goTo(neutralPoints[CentreDot], 70);
    //         setMove(runningSpeed, robotAngle, 0);
    //     }
    // }
    else {
        avoidLine();
    }
    
    updateDribbler();
    updateKick();
    if (usingDribbler) {
        angleCorrect();
    } else if (goalCorrect && camera.oppGoalVisible) {
        aimGoal();
    } else {
        angleCorrect();
    }
    Serial.print(moveData.speed.val);
    Serial.print(" ");
    Serial.print(moveData.angle.val);
    Serial.print(" ");
    Serial.print(moveData.rotation.val);

    Serial.println();
}

void goalie() {
    // defence program
    dribble = false;

    lineAvoid = false;
    kick = false;
    int goalLeftDist, goalRightDist, centreAngle, distThresh;
    if (robotID == 0) {
        centreAngle = 165;
        goalRightDist = 16;
        goalLeftDist = 26;
        distThresh = 32;

    } else {
        centreAngle = 165;
        goalRightDist = 16;
        goalLeftDist = 26;
        distThresh = 32;
    }

    float goalYDist =
        camera.ownGoalDist * cos(deg2rad(180 - camera.ownGoalAngle + heading));
    float goalXDist =
        camera.ownGoalDist * sin(deg2rad(centreAngle - camera.ownGoalAngle + heading));

    if (goalXDist > -goalRightDist &&
        goalXDist < goalLeftDist && goalYDist < 60) {
        lineTrack = true;
        Serial.println("linetracking");
    } else {
        lineTrack = false;
    }
    
    if (movingSideways) {
        movingSideways = !goTo(sidewaysCoordinate, 70);
    } else {
        if (!camera.ownGoalVisible) {
            runningSpeed = 0;
        } else {
            if (lineData.onLine && ballData.visible && ballData.dist < 60) {
                if (millis() - lastBallAlignTime > 1000) {
                    goalieBallPID.resetIntegral();
                }
                lastBallAlignTime = millis();
                // Serial.println("Aliging to ball");
                // if ball is visible, align to ball
                // since robot is line tracking, simply set target angle to ball
                // angle
                float xCo, yCo;
                xCo = relBall.x / 10;
                // yCo = distThresh - goalYDist;

                robotAngle = mod(polarAngle(0, xCo), 360);
                // target ball angle is zero

                float xDist;
                // limit dist error if ball is past edge of goal
                if (relBall.x > 0) {
                    // ball right of robot
                    xDist = min(goalRightDist + goalXDist, abs(relBall.x) / 10);
                    Serial.print(goalRightDist + goalXDist);
                    Serial.print(" ");
                    Serial.println(abs(relBall.x) / 10);
                } else {
                    // ball left of robot
                    xDist = min(goalLeftDist - goalXDist, abs(relBall.x) / 10);
                    Serial.print(goalLeftDist - goalXDist);
                    Serial.print(" ");
                    Serial.println(abs(relBall.x) / 10);
                }

                float distError = xDist;
                float error = abs(nonReflex(ballData.angle)) * distError;
                runningSpeed = goalieBallPID.update(error);

                // Serial.print("speed:  ");
                // Serial.println(runningSpeed);

                // Serial.print(" Ball Angle: ");
                // Serial.print(ballData.angle);
                // Serial.print(" Ball Distance: ");
                // Serial.println(ballData.dist);
                // Serial.print("Angle: ");
                // Serial.println(robotAngle);
                // if (abs(nonReflex(ballData.angle)) < 1) moveSpeed = 0;

                if (((goalXDist < -goalRightDist) && ballData.angle < 180) ||
                    ((goalXDist > goalLeftDist) && ballData.angle > 180)) {
                    Serial.println("STOPPED");
                    runningSpeed = 0;
                }
                if (ballData.angle > 150 && ballData.angle < 210) {
                    // ball behind robot, suck thumb and cry
                    runningSpeed = 0;
                }

                // set maximum speed limit
                runningSpeed = min(runningSpeed, 90);

            } else {
                if (goalYDist > 60) penaltyAvoid = true;
                // Serial.println("Returning to goal centre");
                // if ball not visible, align to goal centre

                // target goal angle is 180
                float xCo, yCo, error;
                xCo = goalXDist;
                // weighted toward y dist

                yCo = distThresh - goalYDist;
                
                if (lineData.onLine) {
                    error = abs(xCo) * 2;
                    if (xCo > 0) robotAngle = 80;
                    else robotAngle = 280;
                    // angle weighted upwards slightly to prevent getting stuck at sides
                } else {
                    error = sqrt(pow(xCo, 2) + pow(yCo, 2));
                    robotAngle = mod(polarAngle(yCo, xCo), 360);
                }

                // Serial.print("ERROR: ");
                // Serial.println(error);
                // Serial.println(camera.ownGoalAngle);
                // Serial.print("Angle: ");
                // Serial.println(robotAngle);
                runningSpeed = min(goalieGoalPID.update(error), 60);
                if ((!lineData.onLine || millis() - lastInTime < 2000) && goalYDist < 50) {
                    runningSpeed = minSpeed;
                }

                // Serial.print("Speed: ");
                // Serial.println(moveSpeed);
            }
        }
    }

    
    if (penaltyAvoid) {
        Serial.println("AVOIDING PENALTY AREA");
        
        lineAvoid = true;
        lineTrack = false;
        if (lineData.onLine) {
            if (previouslyIn && lineCnt < 3) lineCnt++;
            lastLineTime = millis();
            previouslyIn = false;
        } else {
            previouslyIn = true;
        }
        if (lineCnt >= 3) {
            lineAvoid = false;
            lineTrack = true;
            penaltyAvoid = false;
            lineCnt = 0 ;
        }
    } else {
        lineAvoid = false;
    }

    if (!lineData.onLine) lastInTime = millis();
    
    setMove(runningSpeed, robotAngle, 0);
    lastBallAngle = camera.ballAngle;
    lastBallDist = camera.ballDist;
    Serial.print("speed: ");
    Serial.println(runningSpeed);
    Serial.print("Angle: ");
    Serial.println(robotAngle);
    Serial.print("Own goal ang: ");
    Serial.print(camera.ownGoalAngle);
    Serial.print(" Own goal X: ");
    Serial.print(goalXDist);
    Serial.print(" Own goal Y: ");
    Serial.println(goalYDist);
    printLightData();

    angleCorrect();
    updateDribbler();
    updateKick();
}

void normal() {
    // regular program


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
    // // lineAvoid = false;
    // Serial.print("SPEED: ");
    // Serial.print(moveData.speed.val);
    // Serial.print(" ANGLE: ");
    // Serial.print(moveData.angle.val);
    // Serial.println();
    prevRole = currentRole();
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
        setMove(70, dir, 0);
        angleCorrect();
        sendLayer1();
        //  dribble = true;
        //  updateDribbler();
    }
}

void tuneCompassPID() {
    updateAllData();
    setMove(0, 0, 0);
    angleCorrect();
    sendLayer1();
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
    EEPROM.write(EEPROM_ID_ADDR, 1);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    defaultRole = robotID == 0 ? Role::defend : Role::attack;
    // roleSwitching = false;
    if (robotID == 1) minSpeed = 35;
    roleSwitching = true;
    movingSideways = false;
    // defaultRole = Role::defend;
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
    startTime = millis();
}
long tmp = 0;
void loop() {
    // lineAvoid = true;
    // setMove(0,0,0);
    // sendLayer1();
    // updateAllData();
    //
    // printLightData();
    // lineAvoid = true;
    // avoidLine();
    // sendLayer1();
    // dribble = true;
    // updateDribbler();
    printLightData();
    normal();
    Serial.print("LIGHTGATE: ");
    Serial.println(readLightGate());

    // updateAllData();
    // setMove(60,0,0);
    // angleCorrect();
    // sendLayer1();

    // bbox.printTOF();
    // updateAllData();
    // printIMU();
    // setMove(50,0,0);
    // sendLayer1();

    // goTo(GoalieCentre, 50);
    // setMove(runningSpeed, robotAngle, 0);
    // angleCorrect();
    // sendLayer1();
    // lineAvoid = true;
    // sendLayer1();

    // if (bt.isConnected) {
    //     Serial.println("CONNECTED");
    // } else {
    //     Serial.println("DISCONNECTED");
    // }
}