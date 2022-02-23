#include "Bluetooth.h"


void Bluetooth::init() {
  BTSerial.begin(BLUETOOTH_BAUD);
}

void Bluetooth::send() {
  BTSerial.write(BLUETOOTH_SYNC_BYTE);

  // fill buffer and send
  BTBuffer.vals[0] = ownData.ballData.x;
  BTBuffer.vals[1] = ownData.ballData.y;
  BTBuffer.vals[2] = ownData.robotPos.x;
  BTBuffer.vals[3] = ownData.robotPos.y;
  BTBuffer.b[8] = ownData.ballData.visible;
  BTBuffer.b[9] = ownData.onField;
  BTBuffer.b[10] = ownData.playMode;

  BTSerial.write(BTBuffer, sizeof(BTBuffer.b));
  
}

void Bluetooth::receive() {
  bool nothingRecieved = true;
  while (BTSerial.available() >= BLUETOOTH_PACKET_SIZE) {
    uint8_t syncByte = BTSerial.read();
    if (syncByte == BLUETOOTH_SYNC_BYTE) {
        for (int i = 0; i < BLUETOOTH_PACKET_SIZE - 1; i++) {
            BTBuffer.b[i] = BTSerial.read();
        }
        otherData.ballData = BallData(BTBuffer.vals[0], BTBuffer.vals[1], (bool) BTBuffer.b[8]);
        otherData.robotPos = Point(BTBuffer.vals[2], BTBuffer.vals[3]);
        otherData.onField = (bool) BTBuffer.b[9];
        otherData.playMode =  static_cast<PlayMode> BTBuffer.b[10];
        timer.update();
      
    }
    nothingRecieved = false;
  }



void Bluetooth::update(BluetoothData data) {
  ownData = data;
  send();
  receive();
  
}