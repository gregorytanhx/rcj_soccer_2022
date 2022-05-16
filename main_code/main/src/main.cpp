#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

void striker() {
   
    bool goalCorrect = false;
    if (movingSideways) {
        movingSideways = !goTo(sidewaysCoordinate, 100);
    } else if (ballData.captured) {
        if (usingDribbler) {
            if (camera.oppGoalVisible) {
                attackState = chasingGoal;
                // camera.printData();
                Serial.println("TRACKING GOAL");
                Serial.print("Goal Angle: ");
                Serial.print(camera.oppGoalAngle);
                Serial.print(" Goal Dist: ");
                Serial.print(camera.oppGoalDist);
                Serial.println();
                // max speed when aiming for goal
                // runningSpeed = 80;
                trackGoal();
                // turn off front dribbler + turn on kicker if facing front goal
                // and 50cm away
                if (camera.oppGoalDist <= 48 &&
                    ((camera.oppGoalAngle - heading) < 20 || (camera.oppGoalAngle - heading) > 340) &&
                    (robotAngle < 30 || robotAngle > 330)) {
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
        if ((camera.ballAngle < 60 || camera.ballAngle > 300) &&
            camera.ballDist < 60) {
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

    int distThresh = 41;

    // if (camera.oppGoalDist < distThresh) {
    //     goTo(neutralPoints[CentreDot], 70);
    // }
    // avoid fully entering penalty area

    setMove(runningSpeed, robotAngle, 0);
    slowDown();
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
    Serial.print(moveData.speed.val);
    Serial.print(" ");
    Serial.print(moveData.angle.val);
    Serial.print(" ");
    Serial.print(moveData.rotation.val);

    Serial.println();
}

void goalie() {
    // defence program
    lineTrack = true;
    lineAvoid = false;
    int goalLeftDist, goalRightDist, centreAngle, distThresh;
    if (robotID == 0) {
        centreAngle = 165;
        goalRightDist = 18;
        goalLeftDist = 20;
        if (camera.ownGoalAngle > centreAngle)
            distThresh = 31;
        else
            distThresh = 34;
    } else {
        centreAngle = 165;
        goalRightDist = 18;
        goalLeftDist = 20;
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
    // if (millis() - lastChargeTime > 10000) {
    //     if (camera.ballVisible && camera.newData &&
    //         abs(lastBallAngle - camera.ballAngle) > 5 &&
    //         abs(lastBallDist - camera.ballDist) > 20) {
    //         lastBallMoveTime = millis();
    //     }
    //     if (millis() - lastBallMoveTime > 3000 && goalieAttack) {
    //         Serial.println("Charging");
    //         // goalie charges to score if ball has not moved for some time
    //         goalieCharge = true;
    //         goalieChargeTimer = millis();
    //         lineTrack = false;
    //         // move off white line first
    //         while (lineData.onLine) {
    //             updateAllData();
    //             setMove(60, 0, 0);
    //             angleCorrect();
    //             sendLayer1();
    //         }
    //         moveSpeed = 0;
    //         robotAngle = 0;
    //     }
    // }

    if (goalYDist < 40 && lineData.onLine && ballData.visible &&
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
        float xDist;
        // limit dist error if ball is past edge of goal
        if (relBall.x > 0) {
            // ball right of robot
            xDist = min(goalRightDist + goalXDist, abs(relBall.x) /10);
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

        float distError = (80 / (abs(relBall.y) / 10)) + xDist;
        float error = abs(nonReflex(ballData.angle)) * distError;
        moveSpeed = goalieBallPID.update(error);

        Serial.print("speed:  ");
        Serial.println(moveSpeed);

        // Serial.print(" Ball Angle: ");
        // Serial.print(ballData.angle);
        // Serial.print(" Ball Distance: ");
        // Serial.println(ballData.dist);
        // Serial.print("Angle: ");
        // Serial.println(robotAngle);
        // if (abs(nonReflex(ballData.angle)) < 1) moveSpeed = 0;

        if (((goalXDist < -goalRightDist) && ballData.angle < 180) ||
            ((goalXDist > goalLeftDist) && ballData.angle > 180)) {
            // Serial.println("STOPPED");
            moveSpeed = 0;
        }
        if (ballData.angle > 150 && ballData.angle < 210) {
            // ball behind robot, suck thumb and cry
            moveSpeed = 0;
        }
 
        // set maximum speed limit
        moveSpeed = min(moveSpeed, 100);

    } else {
        // Serial.println("Returning to goal centre");
        // if ball not visible, align to goal centre
      
        // target goal angle is 180
        float xCo, yCo;
        xCo = goalXDist;
        // weighted toward y dist
        yCo = 34 - goalYDist;
        robotAngle = polarAngle(yCo, xCo);
        float error = sqrt(pow(xCo, 2) + pow(yCo * 2, 2));

        // Serial.print("Own goal angle: ");
        // Serial.println(camera.ownGoalAngle);
        // Serial.print("Angle: ");
        // Serial.println(robotAngle);
        moveSpeed = min(goalieGoalPID.update(error), 60);

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
    // // lineAvoid = false;
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
    // defaultRole = robotID == 1 ? Role::defend : Role::attack;
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
    camera.side = Side::facingBlue;

    // camera.side = Side::facingYellow;
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

    normal();
    // bbox.print();
    // bbox.printTOF();
    // updateAllData();
    // Serial.print("Goal Angle: ");
    // Serial.print(camera.oppGoalAngle);
    // Serial.print(" Goal Dist: ");
    // Serial.print(abs(camera.oppGoalDist * cos(deg2rad(camera.oppGoalAngle))));
    // Serial.println();
    // setMove(0,0,0);
    // sendLayer1();
    // dribble = true;
    // updateDribbler();
    // updateAllData();
    // sendLayer1();
    // bbox.printTOF();
    // kick = true;
    // updateKick();
    // dribble = true;
    // updateDribbler();
    // Serial.print("LIGHT GATE: ");
    // Serial.println(readLightGate());
    // tuneCompassPID();

    // moveInCircle();
    // updateAllData();
    // printIMU();
    
    // TODO: ADD FAILURE STATE FOR TOF, USE COMPASS IF TOF DIED
    // bbox.printTOF();
    // printIMU();
    // use serial plotter???
    // tuneCompassPID();
//   moveInCircle();
    // tmp = micros();
    // bbox.printTOF();
    // normal();
    // dribble = true;
    // updateDribbler();
    // tuneCompassPID();
    // moveInCircle();
    // setMove(50,0,0);
    // sendLayer1();
    
    
 
    // printLightData();
  
    // // bbox.printTOF();

    // bt.printData();
    // readLayer1();
    // printLightData();
    // sendLayer1();
}