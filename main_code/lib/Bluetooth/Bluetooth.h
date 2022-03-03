#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <BallData.h>
#include <Point.h>
#include <MyTimer.h>
#include <Role.h>

#define BLUETOOTH_BAUD 115200
#define BLUETOOTH_SYNC_BYTE 71
#define BLUETOOTH_PACKET_SIZE 21
#define BLUETOOTH_LOST_COMMUNICATION_TIME 5000
// robots ping each other every second
#define BLUETOOTH_UPDATE_TIME 1000


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
  float confidence; // confidence of robot position

  BluetoothData() {
      ballData = BallData();
      role = Role::attack;
      onField = true;
      robotPos = Point();
  }

  BluetoothData(BallData ballData, Point robotPos,
                Role role, bool onField)
      : ballData(ballData),
        robotPos(robotPos),
        role(role),
        onField(onField) {}
} BluetoothData;

// bluetooth class for HC05
class Bluetooth {
  public: 
    BTBuffer btBuffer;
    BluetoothData ownData;
    BluetoothData otherData;
    bool isConnected = false;
    bool previouslyConnected = false;
    void init();
    void update(BluetoothData data);

    MyTimer timer = MyTimer(BLUETOOTH_LOST_COMMUNICATION_TIME);
    void send();
    void receive();
};

#endif