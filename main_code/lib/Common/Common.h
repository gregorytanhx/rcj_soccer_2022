#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <math.h>

#define pi 3.1415926536

#define DEBUG
// #define SET_ID
//#define USE_LAYER4_IMU

#define FIELD_WIDTH 1500
#define FIELD_HEIGHT 1900
#define GOALIE_HOME_X 0
#define GOALIE_HOME_Y -760
#define STRIKER_HOME_X 0
#define STRIKER_HOME_Y -200

#define EEPROM_ID_ADDR 0
// robot ID: 0 for striker, 1 for goalie
#define ID 1

#define IMU_CALIB_ADDR 1

#define MIN_SPEED 40
#define SPEED 80
#define MAX_SPEED 110

#define DRIBBLER_UPPER_LIMIT 64
#define DRIBBLER_LOWER_LIMIT 32
#define BALL_DRIBBLE_THRESH 40

// 10cm within target
#define COORD_LEEWAY_DIST 100
#define TOF_CONFIDENCE_THRESH 0.4

#define GOALIE_LEEWAY_DIST 5

#define STM32_BAUD 250000

// send: sent by layer 1, rec: received by layer 1
#define LAYER1_REC_PACKET_SIZE 20
#define LAYER1_SEND_PACKET_SIZE 13
#define LAYER1_REC_SYNC_BYTE 2
#define LAYER1_SEND_SYNC_BYTE 2
#define LAYER4_PACKET_SIZE 9
#define LIGHT_PACKET_SIZE 65
#define LAYER4_SYNC_BYTE 1

#define DRIBBLER_WAIT 3000
#define LIGHT_GATE_THRESH 20

#define L1Serial Serial1
#define L4Serial Serial5
#define CamSerial Serial3
#define BTSerial Serial2
#define IMUSerial Serial4

#define L1CommSerial Serial3
#define L1DebugSerial Serial1
#define L4CommSerial Serial2
#define L4DebugSerial Serial1


float deg2rad(float angle);
float rad2deg(float angle);
float angleDiff(float angle1, float angle2);
float angleAverage(float angle1, float angle2);
float angleBetween(float x, float y) ;
float midAngleBetween(float x, float y);
float distance(float x, float y);
float norm(float val, int max, int min);
int sign(int x);
float mod(float x, float y);
float nonReflex(float angle);
bool angleIsInside(float angleBoundCounterClockwise,
                   float angleBoundClockwise,
                   float angleCheck);
float polarAngle(float y, float x);

typedef union floatData {
    float val;
    uint8_t b[4];
} floatData;

typedef union int16Data {
    int16_t val;
    uint8_t b[2];
} int16Data;

typedef union motorBuffer {
    float vals[4];
    uint8_t b[sizeof(vals)];
} motorBuffer;

// struct for line data to be sent over serial
typedef struct LineData {
    floatData lineAngle;
    floatData initialLineAngle;
    floatData chordLength;
    bool onLine;
} LineData;

// struct for compass data to be sent over serial
typedef union CmpVal {
    float val;
    uint8_t b[4];
} CmpVal;


// struct for movement data to be sent from teensy to layer1
typedef struct MoveData {
    floatData speed;
    floatData angle;
    floatData rotation;
    floatData angSpeed;

    MoveData(float moveSpeed, float moveAngle, float moveRotation, float angSpd) {
        speed.val = moveSpeed;
        angle.val = moveAngle;
        rotation.val = moveRotation;
        angSpeed.val = angSpd;
    }
} MoveData;

// struct for TOF values, sent from layer4 to teensy
typedef union TOFBuffer {
    uint16_t vals[4] = {0, 0, 0, 0};
    uint8_t b[8];
} TOFBuffer;

// threshold values for calibrating light sensors
typedef union LightBuffer {
    int16_t vals[32];
    u_int8_t b[64];
} LightBuffer;

#endif