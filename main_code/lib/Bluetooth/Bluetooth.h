#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <BallData.h>
#include <Point.h>

#define BLUETOOTH_BAUD 115200
#define BLUETOOTH_SYNC_BYTE 71
#define BLUETOOTH_PACKET_SIZE 21
#define BLUETOOTH_LOST_COMMUNICATION_TIME 5000


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
      robotPosition = Point();
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
    BluetoothData ownData;
    BluetoothData otherData;
    
    void init();
    void update(BluetoothData data);

  private:
    Timer(BLUETOOTH_LOST_COMMUNICATION_TIME);
    void send();
    void receive();
};

#endif