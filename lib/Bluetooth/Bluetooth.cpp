#include "Bluetooth.h"


void Bluetooth::init() {
  Serial2.begin(BLUETOOTH_BAUD);
}

void Bluetooth::send() {
  Serial2.write(BLUETOOTH_SYNC_BYTE);
  Serial2.write(ownData.ballAngle.b, 4);
  Serial2.write(ownData.ballDist.b, 4);
  Serial2.write(ownData.posX.b, 2);
  Serial2.write(ownData.posY.b, 2);
  Serial2.write(ownData.onField);
  Serial2.write(ownData.playMode);
  Serial2.write(ownData.ballVisible);
}

void Bluetooth::receive() {
  bool nothingRecieved = true;
  while (Serial2.available() >= BLUETOOTH_PACKET_SIZE) {
    uint8_t syncByte = Serial2.read();
    if (syncByte == BLUETOOTH_SYNC_BYTE) {
      for (int i = 0; i < 4; i++) otherData.ballAngle.b[i] = Serial2.read();
      for (int i = 0; i < 4; i++) otherData.ballDist.b[i] = Serial2.read();      
      for (int i = 0; i < 2; i++) otherData.posX.b[i] = Serial2.read();
      for (int i = 0; i < 2; i++) otherData.posY.b[i] = Serial2.read();
      otherData.onField = (bool)Serial2.read();
      otherData.playMode = static_cast<PlayMode>(Serial2.read());
      otherData.ballVisible = (bool)Serial2.read();
      timer.update();
      
    }
    nothingRecieved = false;
  }



void Bluetooth::update(BluetoothData data) {
  ownData = data;
  send();
  receive();
  
}