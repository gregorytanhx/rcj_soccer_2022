#include "Bluetooth.h"

void Bluetooth::begin() { 
    BTSerial.begin(BLUETOOTH_BAUD);
    pinMode(BT_EN_PIN, OUTPUT);
    digitalWrite(BT_EN_PIN, LOW);
}

void Bluetooth::initAT() { 
    BTSerial.begin(BLUETOOTH_BAUD);
    pinMode(BT_EN_PIN, OUTPUT);
    digitalWrite(BT_EN_PIN, HIGH);
}

void Bluetooth::ATmode() {
    while (Serial.available()) {
        BTSerial.write(Serial.read());
    }
    while (BTSerial.available()) {
        Serial.write(BTSerial.read());
    }
}
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
    newData = false;
    while (BTSerial.available() >= sizeof(btBuffer.b) + 1) {
        newData = true;
        uint8_t syncByte = BTSerial.read();
        if (syncByte == BLUETOOTH_SYNC_BYTE) {
            // read into buffer
            for (int i = 0; i < sizeof(btBuffer.b); i++) {
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
    if (millis() - lastSendTime > 100) {
        send();
        lastSendTime = millis();
    }
    receive();
}

void Bluetooth::updateDebug(BBox box, BallData ballData) {
    debugSendData.robotPos = Point(box.x, box.y);
    debugSendData.ballData = ballData;
    debugSendData.width = box.width;
    debugSendData.height = box.height;
    sendDebug();
    // receiveDebug();
}

void Bluetooth::sendDebug() {
    debugSendBuffer.vals[0] = debugSendData.ballData.angle;
    debugSendBuffer.vals[1] = debugSendData.ballData.dist;
    debugSendBuffer.vals[2] = debugSendData.robotPos.getAngle();
    debugSendBuffer.vals[3] = debugSendData.robotPos.getDistance();
    debugSendBuffer.vals[4] = debugSendData.width;
    debugSendBuffer.vals[5] = debugSendData.height;
    BTSerial.write(debugSendBuffer.b, sizeof(debugSendBuffer.b));
    BTSerial.write('\n');
}

void Bluetooth::receiveDebug() {
    while (BTSerial.available() >= DEBUG_PACKET_SIZE) {
        uint8_t syncByte = BTSerial.read();
        if (syncByte == DEBUG_SYNC_BYTE) {
            for (int i = 0; i < DEBUG_PACKET_SIZE - 1; i++) {
                debugRecBuffer.b[i] = BTSerial.read();
            }
            mode = static_cast<DebugMode>(debugRecBuffer.b[28]);
            debugRecData.k = debugRecBuffer.f[0];
            debugRecData.a = debugRecBuffer.f[1];
            debugRecData.b = debugRecBuffer.f[2];
            debugRecData.c = debugRecBuffer.f[3];
            debugRecData.targetPos =
                Point(debugRecBuffer.f[4], debugRecBuffer.f[5]);
            debugRecData.moveAngle = debugRecBuffer.f[6];
        }
    }
}