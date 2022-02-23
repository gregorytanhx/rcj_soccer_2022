#include "Bluetooth.h"


void Bluetooth::init() {
  BTSerial.begin(BLUETOOTH_BAUD);
}

void Bluetooth::send() {
  BTSerial.write(BLUETOOTH_SYNC_BYTE);

  // set up a buffer
  
  BTSerial.write(ownData.ballData., 2);
  BTSerial.write(ownData.ballData.y.b, 2);
  BTSerial.write(ownData.robotPos.x.b, 2);
  BTSerial.write(ownData.robotPos.y.b, 2);
  BTSerial.write(ownData.onField);
  BTSerial.write(ownData.playMode);
  BTSerial.write(ownData.ballData.visible);
}

void Bluetooth::receive() {
  bool nothingRecieved = true;
  while (BTSerial.available() >= BLUETOOTH_PACKET_SIZE) {
    uint8_t syncByte = BTSerial.read();
    if (syncByte == BLUETOOTH_SYNC_BYTE) {
        for (int i = 0; i < 11; i++) {
            BTbuffer.b[i] = BTSerial.read();
        }
        otherData.ballData = BallData()
        timer.update();
      
    }
    nothingRecieved = false;
  }



void Bluetooth::update(BluetoothData data) {
  ownData = data;
  send();
  receive();
  
}