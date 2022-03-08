#include "Bluetooth.h"

void Bluetooth::init() { BTSerial.begin(BLUETOOTH_BAUD); }

void Bluetooth::send() {
    // fill buffer and send

    // 2 x 4 bytes for floats
    btBuffer.f[0] = ownData.ballData.dist;
    btBuffer.f[1] = ownData.confidence;

    // 4 x 2 bytes for int16s
    btBuffer.vals[4] = ownData.ballData.x;
    btBuffer.vals[5] = ownData.ballData.y;
    btBuffer.vals[6] = ownData.robotPos.x;
    btBuffer.vals[7] = ownData.robotPos.y;

    // 4 bytes for uint8s
    btBuffer.b[16] = ownData.ballData.visible;
    btBuffer.b[17] = ownData.ballData.captured;
    btBuffer.b[18] = ownData.onField;
    btBuffer.b[19] = ownData.role;

    BTSerial.write(BLUETOOTH_SYNC_BYTE);
    BTSerial.write(btBuffer.b, sizeof(btBuffer.b));
}

void Bluetooth::receive() {
    bool nothingRecieved = true;
    while (BTSerial.available() >= BLUETOOTH_PACKET_SIZE) {
        uint8_t syncByte = BTSerial.read();
        if (syncByte == BLUETOOTH_SYNC_BYTE) {
            // read into buffer
            for (int i = 0; i < BLUETOOTH_PACKET_SIZE - 1; i++) {
                btBuffer.b[i] = BTSerial.read();
            }
            // obtain data from buffer
            otherData.ballData = BallData(btBuffer.vals[4], btBuffer.vals[5],
                                          (bool) btBuffer.b[16]);
            otherData.ballData.dist = btBuffer.f[0];
            otherData.robotPos = Point(btBuffer.vals[6], btBuffer.vals[7]);
            otherData.onField = (bool) btBuffer.b[18];
            otherData.role = static_cast <Role> (btBuffer.b[19]);
            otherData.confidence = btBuffer.f[1];
            timer.update();
        }
        nothingRecieved = false;
    }
    isConnected = !nothingRecieved || !timer.timeHasPassed();
    if (isConnected) {
        if (!previouslyConnected) {
            previouslyConnected = true;
        }
    } else {
        otherData = BluetoothData();
    }
}

void Bluetooth::update(BluetoothData data) {
    ownData = data;
    send();
    receive();
}

