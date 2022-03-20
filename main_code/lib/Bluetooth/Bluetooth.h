#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <BallData.h>
#include <Common.h>
#include <Config.h>
#include <Pins.h>
#include <MyTimer.h>
#include <Point.h>
#include <Role.h>

#define BLUETOOTH_BAUD 115200
#define BLUETOOTH_SYNC_BYTE 71
#define BLUETOOTH_PACKET_SIZE 21
#define BLUETOOTH_LOST_COMMUNICATION_TIME 5000
// robots ping each other every second
#define BLUETOOTH_UPDATE_TIME 1000

#define DEBUG_SYNC_BYTE 1
// size of received data
#define DEBUG_PACKET_SIZE 29

// union to serialise and deserialise bluetooth data
typedef union BTBuffer {
    float f[2];
    int16_t vals[8];
    uint8_t b[20];
} BTBuffer;


// struct to hold data for all variables to be sent over bluetooth
typedef struct BluetoothData {
    BallData ballData;
    Point robotPos;
    Role role;
    bool onField;
    float confidence;  // confidence of robot position

    BluetoothData() {
        ballData = BallData();
        role = Role::attack;
        onField = true;
        robotPos = Point();
    }

    BluetoothData(BallData ballData, Point robotPos, Role role, bool onField)
        : ballData(ballData),
          robotPos(robotPos),
          role(role),
          onField(onField) {}
} BluetoothData;

enum DebugMode {
    NormalProgram,
    TestLocalisation,
    TuneBallTrack,
    RemoteControl
}

typedef union DebugSendBuffer {
    int16_t vals[6];
    uint8_t b[12];
} DebugSendBuffer;

// struct to hold variables for debugging
typedef struct DebugSendData {
    BallData ballData;
    Point robotPos;
} DebugSendData;

typedef union DebugRecBuffer {
    float f[7];
    uint8_t b[29];
} DebugRecBuffer;

typedef struct DebugRecData {
    DebugMode mode = NormalProgram;
    Point targetPos;
    float moveAngle;
    float k;
    float a;
    float b;
    float c;
} DebugRecData;


// bluetooth class for HC05
class Bluetooth {
   public:
    BTBuffer btBuffer;
    BluetoothData ownData;
    BluetoothData otherData;
    DebugSendBuffer debugSendBuffer;
    DebugRecBuffer debugRecBuffer;
    DebugSendData debugSendData;
    DebugRecData debugRecData;
    DebugMode mode = NormalProgram;

    bool isConnected = false;
    bool previouslyConnected = false;
    MyTimer timer = MyTimer(BLUETOOTH_LOST_COMMUNICATION_TIME);
    void init();
    void initAT();
    void ATmode();
    void update(BluetoothData data);    
    void send();
    void receive();
    void sendDebug();
    void updateDebug(Point robotPos, BallData ballData);
    void receiveDebug();
};

#endif