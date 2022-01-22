#include <Adafruit_BNO055_t4.h>
#include <Arduino.h>
#include <Bluetooth.h>
#include <Common.h>
#include <Config.h>
#include <EEPROM.h>
#include <Light.h>
#include <Motor.h>
#include <PID.h>
#include <Pins.h>
#include <Point.h>
#include <Wire.h>
#include <utility/imumaths.h>

Light light;
LineData lineData;
MoveData moveData;
Motors motors;

float lastLineAngle = 0;
bool lineTrack = false;

bool hasBall = false;

TOFBuffer tof;
IMU imu;
float heading;

Timer kickerTimer(500);

Role role;
uint8_t robotID;

void dribble() { analogWrite(DRIBBLER_PIN, 64); }

void kick() {
    if (kickerTimer.timeHasPassed()) {
        digitalWriteFast(KICKER_PIN, HIGH);
    }
}

void setMove(float speed, float angle, float rotation) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
}

void sendLayer1() {
    L1Serial.write(LAYER1_REC_SYNC_BYTE);
    L1Serial.write(moveData.speed.b, 4);
    L1Serial.write(moveData.angle.b, 4);
    L1Serial.write(moveData.rotation.b, 4);
    L1Serial.write(lineTrack);
}

void readLayer1() {
    while (L1Serial.available() >= LAYER1_SEND_PACKET_SIZE) {
        uint8_t syncByte = L1Serial.read();
        if (syncByte == LAYER1_REC_SYNC_BYTE) {
            for (int i = 0; i < 4; i++) {
                lineData.lineAngle.b[i] = L1Serial.read();
            }
            for (int i = 0; i < 4; i++) {
                lineData.chordLength.b[i] = L1Serial.read();
            }
            lineData.onLine = L1Serial.read();
        }
    }

    void readLayer4() {
        while (L4Serial.available() >= LAYER4_PACKET_SIZE) {
            uint8_t syncByte = L4Serial.read();
            if (syncByte == LAYER4_SYNC_BYTE) {
                for (int i = 0; i < LAYER4_PACKET_SIZE - 1; i++) {
                    tof.b[i] = L4Serial.read();
                }
            }
        }
    }

    void processTOF() {
        // TODO
    }

    void trackBall() {
        // TODO
    }

    void aimGoal() {
        // TODO
    }

    void defend() {
        // TODO
    }

    bool shouldSwitchRoles() { return (role == Role::goalie && hasBall) }
    // void lineTrack(float target) {
    //   float angle = nonReflex(line.getClosestAngle(target));
    //   // use PID to control speed of correction
    //   float correction = lineTrackPID.update(angle - target);

    //   motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
    // }

    // void getCameraCoords() {
    //   Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
    //   Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

    //   int vecX = (oppGoalVec.x + ownGoalVec.x) / 2;
    //   int vecY = (oppGoalVec.y + oppGoalVec.y) / 2;

    //   Point centre(vecX, vecY);

    // }

    void setup() {
#ifdef SET_ID
        EEPROM.write(EEPROM_ID_ADDR, ID)
#endif

            robotID = EEPROM.read(EEPROM_ID_ADDR);
        if (robotID)
            role = Role::striker;
        else
            role = Role::goalie;

#ifdef DEBUG
        Serial.begin(9600);
#endif

        L1Serial.begin(STM32_BAUD);
        L4Serial.begin(STM32_BAUD);
        CamSerial.begin(CAMERA_BAUD);
        BluetoothSerial.begin(BLUETOOTH_BAUD);

        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);

        imu.init();

        pinMode(KICKER_PIN, OUTPUT);
        pinMode(DRIBBLER_PIN, OUTPUT);
        analogWriteFrequency(DRIBBLER_PIN, 1000);
        analogWrite(DRIBBLER_PIN, 32);
        delay(DRIBBLER_WAIT);
    }

    void loop() {
        // put your main code here, to run repeatedly:
        camera.read();
        camera.process();
        heading = imu.read();
        readLayer4();

        if (analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH) {
            hasBall = true;
        }

        if (role == striker) {
            if (hasBall) {
                aimGoal();
            } else {
                trackBall();
            }
        } else {
            defend();
        }

        sendLayer1();
        readLayer1();
    }