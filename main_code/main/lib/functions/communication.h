#ifndef COMM_H
#define COMM_H

#include <communication.h>
#include <declarations.h>
#include <movement.h>

void sendLayer1() {
    // use start and end sync bytes to minimise errors
    L1Serial.write(LAYER1_SYNC_BYTE_START);
    L1Serial.write(moveData.speed.b, 4);
    L1Serial.write(moveData.angle.b, 4);
    L1Serial.write(moveData.rotation.b, 4);
    L1Serial.write(moveData.angSpeed.b, 4);
    L1Serial.write((uint8_t)lineTrack);
    L1Serial.write((uint8_t)lineAvoid);
    L1Serial.write(LAYER1_SYNC_BYTE_END);
    L1Serial.flush();
}

void readLayer1() {
    while (L1Serial.available() >= 14) {
        uint8_t syncByte = L1Serial.read();
        if (syncByte == LAYER1_SEND_SYNC_BYTE) {
            for (int i = 0; i < 4; i++) {
                lineData.lineAngle.b[i] = L1Serial.read();
            }
            for (int i = 0; i < 4; i++) {
                lineData.initialLineAngle.b[i] = L1Serial.read();
            }
            for (int i = 0; i < 4; i++) {
                lineData.chordLength.b[i] = L1Serial.read();
            }
            lineData.onLine = L1Serial.read();
        }
    }
}

void calibIMU() {
    uint8_t sys, gyro, accel, mag = 0;
    while (gyro != 3 || accel != 3) {
        while (IMUSerial.available() >= 5) {
            uint8_t syncByte = IMUSerial.read();
            if (syncByte == IMU_SYNC_BYTE) {
                sys = IMUSerial.read();
                gyro = IMUSerial.read();
                accel = IMUSerial.read();
                mag = IMUSerial.read();
            }
        }
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(sys);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);
        Serial.println("--");
    }
}

void readIMU() {
    while (IMUSerial.available() >= 3) {
        uint8_t syncByte = IMUSerial.read();
        if (syncByte == IMU_SYNC_BYTE) {
            for (int i = 0; i < 2; i++) {
                cmpVal.b[i] = IMUSerial.read();
            }
        }
        heading = (float)cmpVal.val / 100;
        cmpCorrection = cmpPID.update(-heading);
        // Serial.print(-heading);
        // Serial.print("  ");
        // Serial.print(cmpCorrection);
        // Serial.print("  ");
        // Serial.print(0);
        // Serial.println();
    }
}

void printIMU() {
    Serial.print("Heading: ");
    Serial.println(heading);
}

void printLightData() {
    if (lineData.onLine) {
        Serial.println("Line Detected");
        Serial.print("Line Angle: ");
        Serial.print(lineData.lineAngle.val);
        Serial.print(" Chord Length: ");
        Serial.print(lineData.chordLength.val);
        Serial.println();
    } else {
        Serial.println("Line Not Detected");
    }
}

bool readTOF() {
    bool newData = false;
    while (L4Serial.available() >= LAYER4_PACKET_SIZE) {
        newData = true;
        uint8_t syncByte = L4Serial.read();
        if (syncByte == LAYER4_SYNC_BYTE) {
            for (int i = 0; i < LAYER4_PACKET_SIZE - 1; i++) {
                tof.b[i] = L4Serial.read();
            }
        }
    }
    return newData;
}

Role currentRole() {
    if (roleSwitching) {
        // will tell the robot if its supposed to attack or defend
        if (bt.isConnected) {
            // if role is undecided, pick default, else return the role that has
            // been previously assigned
            return role == Role::undecided ? defaultRole : role;
        } else {
            // if other bot disconnects or robot is only one on field, default
            // to striker
            Serial.println("ATTACK");
            return Role::attack;
        }
    } else {
        return defaultRole;
    }
}

bool shouldSwitchRoles(BluetoothData attackerData, BluetoothData defenderData) {
    // switch roles if goalie has ball or ball is much closer to goalie or
    // striker went out out field
    return (defenderData.ballData.captured &&
            !attackerData.ballData.captured) ||
           (defenderData.ballData.visible && attackerData.ballData.visible &&
            (defenderData.ballData.angle < 15 ||
             defenderData.ballData.angle > 345) &&
            (defenderData.ballData.dist < 25) &&
            (attackerData.ballData.dist - defenderData.ballData.dist > 10 ||
             ((attackerData.ballData.angle > 90 &&
               attackerData.ballData.angle < 270) &&
              attackerData.ballData.dist > 30)));
}

void updateRole() {
    Role previousRole = role;

    if (role == Role::undecided) {
        // Undecided role, pick default unless the other robot has a role

        if (bt.otherData.role == Role::undecided) {
            role = defaultRole;
        } else {
            // take the other role based on the other robot's role
            role =
                bt.otherData.role == Role::attack ? Role::defend : Role::attack;
        }
    } else if (robotID == 1) {
        // Robot ID 1 (default striker) decides on role
        Serial.println("HERE");
        BluetoothData attackerData =
            role == Role::attack ? btData : bt.otherData;
        BluetoothData defenderData =
            role == Role::defend ? btData : bt.otherData;
        if (role == Role::attack) {
            Serial.println("ATTACK");
        } else if (role == Role::defend) {
            Serial.println("DEFENCE");
        }
        // Serial.print("Goalie ball angle: ");
        // Serial.print(defenderData.ballData.angle);
        // Serial.print(" Striker ball angle: ");
        // Serial.print(attackerData.ballData.angle);
        // Serial.print(" Goalie ball dist: ");
        // Serial.print(defenderData.ballData.dist);
        // Serial.print(" Striker ball dist: ");
        // Serial.print(attackerData.ballData.dist);

        if (shouldSwitchRoles(attackerData, defenderData) &&
            millis() - lastSwitchTime > 5000 && millis() - startTime > 5000) {
            Serial.println("SWITCH");
            lastSwitchTime = millis();
            role = role == Role::attack ? Role::defend : Role::attack;
        }

    } else {
        // Robot ID 1 is always the the opposite of the other robot
        role = bt.otherData.role == Role::attack ? Role::defend : Role::attack;
    }

    if (role != previousRole && previousRole == Role::attack) {
        // If switched to defender, move to the side to prevent collision
        movingSideways = true;
        sidewaysCoordinate = Point(200 * sign(botCoords.x), botCoords.y);
        penaltyAvoid = true;
    } else {
        movingSideways = false;
    }
}

void updateBluetooth() {
    btData = BluetoothData(ballData, botCoords, role);
    bt.update(btData);
    if (bt.isConnected && roleSwitching) {
        updateRole();
    }
}

void updateDebug() { bt.updateDebug(bbox, ballData); }

void updateAllData() {
    readIMU();
    readLayer1();
    if (readTOF()) updatePosition();
    // updateDebug();
    camera.update();
    updateBallData();
    updateBluetooth();
}

#endif