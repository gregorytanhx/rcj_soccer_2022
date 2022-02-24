#include "Bluetooth.h"

void Bluetooth::init() { BTSerial.begin(BLUETOOTH_BAUD); }

void Bluetooth::send() {
    // fill buffer and send

    // 2 x 4 bytes for floats
    BTBuffer.f[0] = ownData.ballData.dist;
    BTBuffer.f[1] = ownData.confidence;

    // 4 x 2 bytes for int16s
    BTBuffer.vals[4] = ownData.ballData.x;
    BTBuffer.vals[5] = ownData.ballData.y;
    BTBuffer.vals[6] = ownData.robotPos.x;
    BTBuffer.vals[7] = ownData.robotPos.y;

    // 4 bytes for uint8s
    BTBuffer.b[16] = ownData.ballData.visible;
    BTBuffer.b[17] = ownData.ballData.captured;
    BTBuffer.b[18] = ownData.onField;
    BTBuffer.b[19] = ownData.role;

    BTSerial.write(BLUETOOTH_SYNC_BYTE);
    BTSerial.write(BTBuffer, sizeof(BTBuffer.b));
}

void Bluetooth::receive() {
    bool nothingRecieved = true;
    while (BTSerial.available() >= BLUETOOTH_PACKET_SIZE) {
        uint8_t syncByte = BTSerial.read();
        if (syncByte == BLUETOOTH_SYNC_BYTE) {
            // read into buffer
            for (int i = 0; i < BLUETOOTH_PACKET_SIZE - 1; i++) {
                BTBuffer.b[i] = BTSerial.read();
            }
            // obtain data from buffer
            otherData.ballData = BallData(BTBuffer.vals[4], BTBuffer.vals[5],
                                          (bool)BTBuffer.b[16]);
            otherData.ballData.dist = BTBuffer.f[0];
            otherData.robotPos = Point(BTBuffer.vals[6], BTBuffer.vals[7]);
            otherData.onField = (bool)BTBuffer.b[18];
            otherData.role = static_cast<Role> BTBuffer.b[19];
            otherData.confidence = BTBuffer.f[1];
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

// TODO: Add functions for bluetooth debugging
// current plans: 
// send over ball data and position data
// receive target position for movement and ball curve tuning parameters
