#include <communication.h>
#include <declarations.h>
#include <localisation.h>
#include <movement.h>

void normal() {
    updateAllData();
    if (currentRole() == Role::attack) {
        // ball captured
        if (ballData.captured) {
            trackGoal();
            // turn off front dribbler + turn on kicker if facing front goal and
            // 50cm away
            if (camera.oppDist <= 50 &&
                (camera.oppAngle < 30 || camera.oppAngle > 330) &&
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
        } else if () {
            trackBall();
            // turn on front dribbler if ball within +-50 deg of front AND
            // closer than 50 cm
            if ((camera.ballAngle < 50 || camera.ballAngle > 310) && camera.ballDist < 50) {
                dribble = true;
            }
        }

        else {
            goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
        }
    } else {
        if (millis()  - lastBallTime < 500) {
            guardGoal();
        } else {
            goTo(Point(GOALIE_HOME_X, GOALIE_HOME_Y));
        }
    }
    updateKick();
    updateDribbler();
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

void robustness() {
    // TBD
}

void precisionMovement() {
    int points[11] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    int cnt = 0;
    lineAvoid = false;
    while (cnt < 11) {
        updateAll();
        goTo(neutralPoints[points[cnt]]);
        if (reachedPoint(neutralPoints[points[cnt]])) {
            lineTrack = false;
            long timer = millis();
            while (millis() - timer < 1500) {
                updateAll();
                setMove(0, 0, 0);
                sendLayer1();
            }
            cnt++;
            // Point transitionPoint;
            // bool transition = true;
            // switch (points[cnt]) {
            //     case TopLeftCorner:
            //         transitionPoint = TopLeftDot;
            //         break;
            //     case TopRightCorner:
            //         transitionPoint = TopRightDot;
            //         break;
            //     case BottomRightCorner:
            //         transitionPoint = BottomRightDot;
            //         break;
            //     case BottomLeftCorner:
            //         transitionPoint = BottomLeftDot;
            //         break;
            //     default:
            //         transition = false;
            // }

            // if (transition) {
            //     // move to at least 30 cm within transition point
            //     while (!reachedPoint(transitionPoint, 300)) {
            //         updateAll();
            //         goTo(transitionPoint);
            //         angleCorrect();
            //         sendLayer1();
            //     }
            // }
        }
        angleCorrect();
        sendLayer1();
    }
    while (1) {
        setMove(0, 0, 0);
        sendLayer1();
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