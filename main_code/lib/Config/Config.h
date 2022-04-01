#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define DEBUG
// #define SET_ID
//#define USE_LAYER4_IMU


#define FIELD_WIDTH 1820
#define FIELD_HEIGHT 2430
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
#define MOTOR_MULT 1.0154266118857451

// kick ball when 40cm to the goal
#define KICK_DISTANCE_THRES 40 

#define CMP_KP 0.8

// 10cm within target
#define COORD_LEEWAY_DIST 100
#define TOF_CONFIDENCE_THRESH 0.4

#define GOALIE_LEEWAY_DIST 5

#define CAMERA_PACKET_SIZE 17
#define CAMERA_SYNC_BYTE 42
#define CAMERA_BAUD 1000000

#define STM32_BAUD 250000

#define OFFSET_MULT 0.95
#define BALL_MULT_A 1
#define BALL_MULT_B 1

// send: sent by layer 1, rec: received by layer 1
#define LAYER1_REC_PACKET_SIZE 16
#define LAYER1_SEND_PACKET_SIZE 13
#define LAYER1_REC_SYNC_BYTE 2
#define LAYER1_SEND_SYNC_BYTE 2
#define LAYER4_PACKET_SIZE 9
#define LIGHT_PACKET_SIZE 65

// tof settings
#define TIME_BUDGET 100
#define IMP 33
#define LAYER4_SYNC_BYTE 1

#define DRIBBLER_WAIT 3000
#define LIGHT_GATE_THRESH 20


#define L1Serial Serial1
#define L4Serial Serial5
#define CamSerial Serial3
#define BTSerial Serial2

#define L1CommSerial Serial3
#define L1DebugSerial Serial1
#define L4CommSerial Serial2
#define L4DebugSerial Serial1

#endif