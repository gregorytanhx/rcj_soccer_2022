
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define SET_ID 0
#define EEPROM_ID_ADDR 0
// robot ID: 0 for striker, 1 for goalie 
#define ID 0


#define CMP_KP = 0.8;

#define COORD_KP = 2;
#define COORD_KI = 0;
#define COORD_KD = 1;

#define LINE_TRACK_KP 2
#define LINE_TRACK_KI 0
#define LINE_TRACK_KD 1
#define LINE_TRACK_SPEED 60

#define CAMERA_PACKET_SIZE 13
#define CAMERA_SYNC_BYTE 0x80
#define CAMERA_BAUD 2000000

#define STM32_BAUD 2000000

// send: sent by layer 1, rec: received by layer 1
#define LAYER1_REC_PACKET_SIZE 14
#define LAYER1_SEND_PACKET_SIZE 9
#define LAYER1_REC_SYNC_BYTE 2
#define LAYER1_SEND_SYNC_BYTE 2

// tof settings
#define TIME_BUDGET 33
#define IMP 33
#define LAYER4_SYNC_BYTE 1

#define DRIBBLER_WAIT 3000
#define LIGHT_GATE_THRESH 100

#endif