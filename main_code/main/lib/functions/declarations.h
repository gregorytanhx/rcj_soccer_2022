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
bool usingDribbler = true;
bool lineTrack = false;
bool lineAvoid = true;
bool hasBall = false;
bool calibrate = false;
bool doneCalibrating = false;
bool kick = false;
bool dribble = false;
bool previouslyCharging = false;
bool penaltyAvoid = false;
bool goalieAttack = false;
bool kicked = false;
bool previouslyIn = true;
long lastChargeTime, lastBallMoveTime, lastKickTime, lastGateTime, lastBallTime;
long lastLineTime, lastInTime, dribblerOnTime, lastSwitchTime, lastDribbleTime;
long lastOutTime, lastBallAlignTime, lastCoordTime;
long startTime;

int ballCnt = 0;
float lastBallAngle;
float lastBallDist;
float cmpCorrection;
bool goalieCharge = false;
long goalieChargeTimer;
int lastDist;
int shootCnt = 0;
int distCnt = 0;
int lineCnt;
float outBallAngle, outBallDist;

TOFBuffer tof;
// IMU cmp(&Wire1);
int runningSpeed;
int minSpeed = 30;
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
bool inPenalty = false;
bool previouslyCaptured = false;
Role role = Role::undecided;
Role defaultRole;
Role prevRole;
uint8_t robotID;

Point botCoords(0, 0);
Point relBall(0, 0);
Point absBall(0, 0);
Point sidewaysCoordinate(0, 0);


float coord_kp = 0.3;
float coord_ki = 0;
float coord_kd = 1;
PID coordPIDX(coord_kp, coord_ki, coord_kd, 0.8);
PID coordPIDY(coord_kp, coord_ki, coord_kd, 0.8);
// no dribbler
// PID cmpPID(0.18, 0.15, 20.5, 0.4);
// 0.09, 0.5
// white bot vals (wif dribbler=)
PID cmpPID(0.19, 0.13, 43.5, 0.4);
PID goalieBallPID(0.4, 0.02, 10, 0.8);
PID goalieGoalPID(2.4, 0.02, 45, 0.8);
PID camAngPID(0.1, 0, 0.3);

// initialise neutral point coordinates
// each point is an x and y coordinate with respect to field centre
// Point neutralPoints[] = {Point(-370, 550),  Point(350, 520),  Point(0, 0),
//                          Point(-350, -400), Point(370, -400), Point(0, -660),
//                          Point(0, 660),     Point(965, -660), Point(965,
//                          660), Point(-965, -660), Point(-965, 660)};
// for camera only
// Point neutralPoints[] = {Point(-135, ),  Point(200, 500),  Point(0, -100),
//                          Point(-350, -500), Point(250, -300), Point(0, -660),
//                          Point(0, 660),     Point(965, -660), Point(965,
//                          660), Point(-965, -660), Point(-965, 660)};
// for TOF only
Point neutralPoints[] = {Point(-260, 350),  Point(280, 370),  Point(50, 0),
                         Point(-270, -300), Point(295, -390), Point(-500, 0),
                         Point(500, 0),     Point(965, -0),   Point(965, 660),
                         Point(-965, -660)};

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
    BottomRightCorner, 
    
};

Point GoalieCentre(50, -350);

enum state {
    chasingBall, 
    chasingGoal, 
    returningToCentre
};

state attackState; 

int readLightGate() { return lightGateVal.reading(analogRead(LIGHT_GATE_PIN)); }

void setMove(float speed, float angle, float rotation, float angSpeed = -1.0) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
    moveData.angSpeed.val = angSpeed;
}

#endif
