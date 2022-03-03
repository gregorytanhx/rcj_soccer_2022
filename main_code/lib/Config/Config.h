#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define DEBUG
#define SET_ID


#define FIELD_WIDTH 1320
#define FIELD_HEIGHT 1920
#define GOALIE_HOME_X 0
#define GOALIE_HOME_Y -760
#define STRIKER_HOME_X 0
#define STRIKER_HOME_Y -200

#define EEPROM_ID_ADDR 0
// robot ID: 0 for striker, 1 for goalie
#define ID 0

#define MIN_SPEED 40
#define SPEED 80
#define MAX_SPEED 100

#define IMU_KP 0.8

#define COORD_KP 2
#define COORD_KI 0
#define COORD_KD 1
#define COORD_LEEWAY_DIST 5 

#define LINE_TRACK_KP 2
#define LINE_TRACK_KI 0
#define LINE_TRACK_KD 1
#define LINE_TRACK_SPEED 60

#define GOALIE_KP 1
#define GOALIE_KI 0
#define GOALIE_KD 0

#define GOALIE_LEEWAY_DIST 5

#define CAMERA_PACKET_SIZE 13
#define CAMERA_SYNC_BYTE 42
#define CAMERA_BAUD 115200

#define STM32_BAUD 250000
#define BLUETOOTH_BAUD 115200

#define OFFSET_MULT 0.95
#define BALL_MULT_A 1
#define BALL_MULT_B 1


// send: sent by layer 1, rec: received by layer 1
#define LAYER1_REC_PACKET_SIZE 15
#define LAYER1_SEND_PACKET_SIZE 9
#define LAYER1_REC_SYNC_BYTE 2
#define LAYER1_SEND_SYNC_BYTE 2
#define LAYER4_PACKET_SIZE 9

// tof settings
#define TIME_BUDGET 33
#define IMP 33
#define LAYER4_SYNC_BYTE 1

#define DRIBBLER_WAIT 3000
#define LIGHT_GATE_THRESH 100

#define L1Serial Serial1
#define L4Serial Serial2
#define CamSerial Serial3
#define BTSerial Serial4

#define L1CommSerial Serial3
#define L1DebugSerial Serial1
#define L4CommSerial Serial1
#define L4DebugSerial Serial2

#endif