#include <Adafruit_BNO055_t4.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BBox.h>
#include <Bluetooth.h>
#include <Camera.h>
#include <Common.h>
#include <Config.h>
#include <EEPROM.h>
#include <IMU.h>
#include <PID.h>
#include <MyTimer.h>
#include <Pins.h>
#include <Point.h>
#include <Role.h>
#include <Wire.h>
#include <functions.h>


// TODO: triangulate position based on coords of both goals
// TODO: if only one goal visible, use that goal
// TODO: if goalie, use own goal only

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
    //cmp.begin();
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
        //bbox.print();
        bbox.printTOF();
        String dir[4] = {"Front", "Left", "Back", "Right"};
        for (int i = 0; i < 4; i++) {
            Serial.print(dir[i] + "Raw : ");
            Serial.print(tof.vals[i]);
            Serial.print("  ");
        }
        Serial.println();
    }

    //goTo(neutralPoints[CentreDot]);
    // angleCorrect();
    // sendLayer1();
    // bbox.print();

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

    // camera.update():

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
    //             kick();
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