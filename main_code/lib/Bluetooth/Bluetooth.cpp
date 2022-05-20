#include "Bluetooth.h"

void Bluetooth::begin() {
    BTSerial.begin(BLUETOOTH_BAUD);
    pinMode(BT_EN_PIN, OUTPUT);
    digitalWrite(BT_EN_PIN, LOW);
    pinMode(BT_RESET_PIN, OUTPUT);
    digitalWrite(BT_RESET_PIN, LOW);
    delay(200);
    digitalWrite(BT_RESET_PIN, HIGH);
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

    // 6 x 2 bytes for int16s
    btBuffer.vals[0] = (int)(ownData.confidence * 100);
    btBuffer.vals[1] = (int)(ownData.ballData.angle * 100);
    btBuffer.vals[2] = (int)(ownData.ballData.dist * 100);
    
    btBuffer.vals[3] = ownData.ballData.x;
    btBuffer.vals[4] = ownData.ballData.y;
    btBuffer.vals[5] = ownData.robotPos.x;
    btBuffer.vals[6] = ownData.robotPos.y;
    

    // 4 bytes for uint8s
    btBuffer.b[14] = ownData.ballData.visible;
    btBuffer.b[15] = ownData.ballData.captured;
    btBuffer.b[16] = ownData.role;

    BTSerial.write(BLUETOOTH_SYNC_BYTE);
    BTSerial.write(btBuffer.b, sizeof(btBuffer.b));
}

void Bluetooth::receive() {
    bool nothingReceived = true;
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
            otherData.ballData = BallData(
                (float) btBuffer.vals[1] / 100, (float) btBuffer.vals[2] / 100, (bool)btBuffer.b[14]);
            otherData.ballData.x = btBuffer.vals[3] ;
            otherData.ballData.y = btBuffer.vals[4] ;
            otherData.ballData.captured = btBuffer.b[15];
            otherData.robotPos = Point(btBuffer.vals[5], btBuffer.vals[6]);
            otherData.role = static_cast<Role>(btBuffer.b[16]);
            otherData.confidence = (float)btBuffer.vals[0] / 100;
            lastReceiveTime = millis();
        }
        nothingReceived = false;
    }

    isConnected = !nothingReceived || !(millis() - lastReceiveTime > BLUETOOTH_LOST_COMMUNICATION_TIME);
    if (isConnected) {
        if (!previouslyConnected) {
            previouslyConnected = true;
        }
    } else {
        otherData = BluetoothData();
    }
}

void Bluetooth::printData() {
    Serial.println("Other Data");
    Serial.print("Ball Angle: ");
    Serial.print(otherData.ballData.angle);
    Serial.print(" Ball Dist: ");
    Serial.print(otherData.ballData.dist);
    Serial.print(" Ball X : ");
    Serial.print(otherData.ballData.x);
    Serial.print(" Ball Y: ");
    Serial.print(otherData.ballData.y);
    Serial.print(" X Pos: ");
    Serial.print(otherData.robotPos.x);
    Serial.print(" Y Pos: ");
    Serial.print(otherData.robotPos.y);

    Serial.println();
}

void Bluetooth::update(BluetoothData data) {
    ownData = data;
    if (millis() - lastSendTime > 10) {
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