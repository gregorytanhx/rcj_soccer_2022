#ifndef FUNCTIONS_H
#define FUNCTIONS_H

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
#include <Pins.h>
#include <Point.h>
#include <Role.h>
#include <Wire.h>

LineData lineData;
MoveData moveData(0, 0, 0);
BallData ballData;
Bluetooth bt;
BluetoothData btData;
Camera camera;
BBox bbox;
LightBuffer lightVals;

float lastLineAngle = 0;
bool lineTrack = false;
bool lineAvoid = true;
bool hasBall = false;
bool calibrate = false;
bool doneCalibrating = false;
bool kick = false;
bool kicked = false;
long lastBallTime = 0;
long lastKickTime = 0;
long lastGateTime = 0;
long lastDribbleTime = 0;

TOFBuffer tof;
IMU cmp(&Wire1);
float heading;
float moveSpeed;
float robotAngle;
float frontTOF, backTOF, leftTOF, rightTOF;

MyTimer kickerTimer(50);
MyTimer bluetoothTimer(BLUETOOTH_UPDATE_TIME);
MyTimer dribblerTimer(2000);

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
PID cmpPID(0.2, 0.001, 0.1);

// initialise neutral point coordinates
// each point is an x and y coordinate with respect to field centre
Point neutralPoints[] = {Point(-350, 515),  Point(350, 515),  Point(0, 0),
                         Point(-350, -515), Point(350, -515), Point(0, -660),
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

bool readLightGate() { return analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH; }

void setMove(float speed, float angle, float rotation) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
}

#endif
