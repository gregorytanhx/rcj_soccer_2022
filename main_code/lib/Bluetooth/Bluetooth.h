#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Common.h>

#define BLUETOOTH_BAUD 115200
#define BLUETOOTH_SYNC_BYTE 71
#define BLUETOOTH_PACKET_SIZE 15
#define BLUETOOTH_LOST_COMMUNICATION_TIME 5000

enum Role {
  striker,
  goalie
};

typedef struct BluetoothData {
  floatData ballAngle;
  floatData ballDist;
  bool ballVisible;
  int16Data posX;
  int16Data posY;
  bool onField;
  PlayMode playMode;
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