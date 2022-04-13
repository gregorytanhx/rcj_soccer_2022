#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Adafruit_BNO055_t4.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BBox.h>
#include <Bluetooth.h>
#include <Camera.h>
#include <Common.h>
#include <EEPROM.h>
// #include <IMU.h>
#include <PID.h>
#include <Pins.h>
#include <Point.h>
#include <Role.h>
#include <Wire.h>
#include <movingAvg.h>

//#define SWITCH_ROLES


LineData lineData;
MoveData moveData(0, 0, 0, 0);
BallData ballData;
Bluetooth bt;
BluetoothData btData;
Camera camera;
BBox bbox;
LightBuffer lightVals;
CmpVal cmpVal;


float lastLineAngle = 0;
bool lineTrack = false;
bool lineAvoid = true;
bool hasBall = false;
bool calibrate = false;
bool doneCalibrating = false;
bool kick = false;
bool dribble = false;
bool previouslyCharging = false;
bool goalieAttack = true;
bool kicked = false;
long lastBallTime = 0;
long lastKickTime = 0;
long lastGateTime = 0;
long lastBallMoveTime = 0;
long lastChargeTime = 0;
long lastDribbleTime = 0;


int ballCnt = 0;
float lastBallAngle;
float lastBallDist;
bool goalieCharge = false;
long goalieChargeTimer;
int lastDist;
int distCnt = 0;

TOFBuffer tof;
// IMU cmp(&Wire1);
float heading;
float moveSpeed;
float robotAngle;
float frontTOF, backTOF, leftTOF, rightTOF;

MyTimer kickerTimer(50);
MyTimer bluetoothTimer(BLUETOOTH_UPDATE_TIME);
MyTimer dribblerTimer(2000);
movingAvg lightGateVal(10);

// decide if bots will switch roles mid match (while both still in)
bool roleSwitching = false;
bool movingSideways = false;
bool onField = true;
Role role = Role::undecided;
Role defaultRole;
uint8_t robotID;

Point botCoords(0, 0);
Point relBallCoords(0, 0);
Point absBallCoords(0, 0);

PID coordPID(0.15, 0, 0.1);
PID cmpPID(0.15, 0, 0.6);

// initialise neutral point coordinates
// each point is an x and y coordinate with respect to field centre
// Point neutralPoints[] = {Point(-370, 550),  Point(350, 520),  Point(0, 0),
//                          Point(-350, -400), Point(370, -400), Point(0, -660),
//                          Point(0, 660),     Point(965, -660), Point(965, 660),
//                          Point(-965, -660), Point(-965, 660)};
// for camera only
Point neutralPoints[] = {Point(-550, 550),  Point(150, 550),  Point(-100, -100),
                        Point(-550, -750), Point(200, -500), Point(0, -660),
                        Point(0, 660),     Point(965, -660), Point(965, 660),
                        Point(-965, -660), Point(-965, 660)};

// enum for neutral points
enum points {
    TopLeftDot,
    TopRightDot,
    CentreDot,
    BottomLeftDot,
    BottomRightDot,
    LeftSide,
    RightSide,
    TopLeftCorner,
    TopRightCorner,
    BottomLeftCorner,
    BottomRightCorner
};

int readLightGate() {
    return lightGateVal.reading(analogRead(LIGHT_GATE_PIN));
    
     }

void setMove(float speed, float angle, float rotation, float angSpeed = -1.0) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
    moveData.angSpeed.val = angSpeed;
}

#endif
