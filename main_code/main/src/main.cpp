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
float robotAngle;

Timer kickerTimer(500);

Role role;
uint8_t robotID;

Point botCoords(0, 0);
Point ballCoords(0, 0);


PID coordPID(COORD_KP, COORD_KI, COORD_KD);
PID goaliePID(GOALIE_KP, GOALIE_KI, GOALIE_KD);

BallData ballData;
Bluetooth bt;
BluetoothData btData;

// initialise neutral point coordinates
// Point neutralPoints[] = {
//     Point(10, 10);
//     ...
// };



void dribble() { 
    // use pwm to control dribbler
    analogWrite(DRIBBLER_PIN, 64); 
}

void kick() {
    if (kickerTimer.timeHasPassed()) {
        digitalWriteFast(KICKER_PIN, HIGH);
    } else {
        digitalWriteFast(KICKER_PIN, LOW);
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
    // TODO: convert TOF distances to bounding box of robot on field
    // use kalman filter to detect if TOF is temporarily blocked?

}

void trackBall() {
    // TODO: tune using simulator
    if (!ballData.visible) {
        Point diff = ballCoords - botCoords;
        ballData.angle = diff.getAngle();
        ballData.dist = diff.getDistance();
    } 
    float ballOffset;
    if (ballData.angle < 180){
        ballOffset = fmin(ballData.angle * 0.96, 90);
    }   else {
        ballOffset = max((360 - ballData.angle) * 0.96, -90);
    } 
    float factor = 1 - ballData.dist / 520;
    float ballMult = fmin(1, 0.0134 * exp(factor * 2.6));

    robotAngle = ballData.angle + ballMult * ballOffset;
    setMove(SPEED, robotAngle, 0);
     
}

void getBallData() {    
    ballData.visible = camera.ballVisible;
    if (ballData.visible) {
        Point tmp(camera.ballAngle, camera.ballDist);
        ballCoords = botCoords + tmp;
        ballData.angle = camera.ballAngle;
        ballData.dist = camera.ballDist;
        ballData.x = ballCoords.x;
        ballData.y = ballCoords.y;
    } else {
        // derive angle and distance of ball relative to robot based on its absolute coordinates
        ballData.x = otherBallCoords.x;
        ballData.y = otherBallCoords.y;
        ballCoords = otherBallCoords - botCoords;
        ballData.angle = ballCoords.getAngle();
        ballData.dist = ballCoords.getDist();
    }
}

void trackGoal() {
    float goalOffset;
    if (camera.oppAngle < 180) {
        goalOffset = fmin(camera.oppAngle, 90);
    } else {
        goalOffset = max((360 - camera.oppAngle), -90);
    }
    float goalMult = 1.5;

    robotAngle = camera.oppAngle + goalMult * goalOffset;
    setMove(SPEED, robotAngle, 0);
}

void defend() {
    float moveSpeed = (abs(ballData.x) < GOALIE_LEEWAY_DIST) ? 0 : max(goaliePID.update(abs(ballData.x)), MIN_SPEED);
    float moveAngle = (ballData.angle > 180) ? -90 : 90;
    lineTrack = true;
}

void goTo(Point target) {
    // determine angle from TOF
    Point tmp = target - botCoords;
    float moveAngle = tmp.getAngle();
    float dist = tmp.getDist();
    float moveSpeed = (dist < COORD_LEEWAY_DIST) ? 0 : max(coordPID.update(dist), MIN_SPEED);

    setMove(moveSpeed, moveAngle, 0);
}

void goToWithCam(Point target) {
    Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
    Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

    // vector pointing to the centre of the field
    Point centre = oppGoalVec + ownGoalVec;
    centre.distance /= 2;
    Point moveVector = centre + target;
    setMove(moveSpeed, moveVector.getAngle(), 0);

}

void updateBluetooth() {
    btData = BluetoothData(....)
}

bool shouldSwitchRoles() { 
    // switch roles if goalie has ball or ball is much closer to goalie or striker went out out field
    return (role == Role::goalie && hasBall)
}

// void getCameraCoords() {
//   Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
//   Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

//   int vecX = (oppGoalVec.x + ownGoalVec.x) / 2;
//   int vecY = (oppGoalVec.y + oppGoalVec.y) / 2;

//   Point centre(vecX, vecY);
// }

void setup() {
#ifdef SET_ID
    EEPROM.write(EEPROM_ID_ADDR, ID);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif

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

    hasBall = analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH;

    if (role == striker) {
        if (hasBall) {
            trackGoal();
        } else {
            trackBall();
        }
    } else {
        defend();
    }

    sendLayer1();
    readLayer1();
}