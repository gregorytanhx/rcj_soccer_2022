#include <declarations.h>
#include <movement.h>
#include <communication.h>
#include <localisation.h>

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
    cnt = 0;
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
    EEPROM.write(EEPROM_ID_ADDR, ID);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    // defaultRole = robotID == 0 ? Role::attack : Role::defend;

#ifdef DEBUG
    Serial.begin(9600);
#endif
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    camera.begin();
    bt.begin();
    // cmp.begin();
    bbox.begin();

    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    pinMode(DRIBBLER_PIN, OUTPUT);
    // analogWriteFrequency(DRIBBLER_PIN, 1000);
    // analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    // delay(DRIBBLER_WAIT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // TODO: Test TOF localisation

    // front, left, back, right
    if (readTOF()) {
        updatePosition();
        updateDebug();
        // bbox.print();
        bbox.printTOF();
        String dir[4] = {"Front", "Left", "Back", "Right"};
        for (int i = 0; i < 4; i++) {
            Serial.print(dir[i] + "Raw : ");
            Serial.print(tof.vals[i]);
            Serial.print("  ");
        }
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

    // if (currentRole() == Role::attack) {
    //     if (ballData.captured) {
    //         trackGoal();
    //         if (camera.oppDist <= KICK_DISTANCE_THRES) {
    //             kick = true;
    //         }
    //     } else if (ballData.visible) {
    //         trackBall();

    //     } else {
    //         goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
    //     }
    // } else {
    //     if (ballData.visible) {
    //         guardGoal();
    //     } else {
    //         goTo(Point(GOALIE_HOME_X, GOALIE_HOME_Y));
    //     }
    // }

    // if (kickerTimer.timeHasPassed()) digitalWriteFast(KICKER_PIN, HIGH);

    // angleCorrect();
    // sendLayer1();
    // readLayer1();
}