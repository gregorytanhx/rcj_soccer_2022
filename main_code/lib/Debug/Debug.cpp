#include "Debug.h"

void Debug::init() {
    BTSerial.begin(38400);
}

void Debug::send() {
    // send polar coordinates instead of cartesian
    
    sendBuffer.f[0] = sendData.ballData.angle;
    sendBuffer.f[1] = sendData.ballData.dist;
    sendBuffer.f[2] = sendData.robotPos.angle;
    sendBuffer.f[3] = sendData.robotPos.dist;

    BTSerial.write(DEBUG_SYNC_BYTE);
    BTSerial.write(sendBuffer, sizeof(sendBuffer.b));
}

void Debug::receive() {
    while (BTSerial.available() >= DEBUG_PACKET_SIZE) {
        uint8_t syncByte = BTSerial.read();
        if (syncByte == DEBUG_SYNC_BYTE) {
            for (int i = 0; i < DEBUG_PACKET_SIZE - 1; i++) {
                recBuffer.b[i] = BTSerial.read();
            }
            mode = static_cast<DebugMode> recBuffer.b[28];
            k = recBuffer.f[0];
            a = recBuffer.f[1];
            b = recBuffer.f[2];
            c = recBuffer.f[3];
            targetPos = Point(recBuffer.f[4], recBuffer.f[5]);
            moveAngle = recBuffer.f[6];
        }

    }
}

void Debug::update() {
    sendData = data;
    send();
    receive();
}