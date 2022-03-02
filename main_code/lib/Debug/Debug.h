#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <BallData.h>
#include <Config.h>
#include <Point.h>
// TODO: Add functions for bluetooth debugging
// current plans:
// send over ball data and position data
// receive target position for movement and ball curve tuning parameters

#define DEBUG_SYNC_BYTE 1
// size of received data
#define DEBUG_PACKET_SIZE 29

enum DebugMode {
    NormalProgram, 
    TestLocalisation, 
    TuneBallTrack, 
    RemoteControl
}

typedef union DebugSendBuffer {
    float f[4]; 
    uint8_t b[16];
} DebugBuffer;

typedef struct DebugSendData {
    BallData ballData;
    Point robotPos;
} DebugData;

typedef union DebugRecBuffer {
    float f[7];
    uint8_t b[29];
} DebugBuffer;

typedef struct DebugRecData {
    DebugMode mode = NormalProgram;
    Point targetPos;
    float moveAngle;
    float k;
    float a;
    float b;
    float c;
} DebugData;

class Debug {
    public:
      DebugSendBuffer sendBuffer;
      DebugSendData sendData;
      DebugRecBuffer recBuffer;
      DebugRecData recData;
      void init();
      void send();
      void receive();
      void update(DebugSendData data);


}
#endif


