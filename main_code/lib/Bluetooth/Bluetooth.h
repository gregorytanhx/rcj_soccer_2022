#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <BallData.h>
#include <Point.h>

#define BLUETOOTH_BAUD 115200
#define BLUETOOTH_SYNC_BYTE 71
#define BLUETOOTH_PACKET_SIZE 12
#define BLUETOOTH_LOST_COMMUNICATION_TIME 5000

typedef union BTbuffer {
    int16_t vals[4];
    uint8_t b[11];
} int16Data;

typedef struct BluetoothData {
  BallData ballData;
  Point robotPos;
  PlayMode playMode;
  bool onField;

  BluetoothData() {
      ballData = BallData();
      playMode = PlayMode::attackMode;
      onField = true;
      robotPosition = Point();
  }

  BluetoothData(BallData ballData, Point robotPos,
                PlayMode playMode, bool onField)
      : ballData(ballData),
        robotPos(robotPos),
        playMode(playMode),
        onField(onField) {}
} BluetoothData;

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