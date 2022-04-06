#ifndef COMM_H
#define COMM_H

#include <declarations.h>
#include <movement.h>
#include <communication.h>

void sendLayer1() {
    L1Serial.write(LAYER1_REC_SYNC_BYTE);
    L1Serial.write(moveData.speed.b, 4);
    L1Serial.write(moveData.angle.b, 4);
    L1Serial.write(moveData.rotation.b, 4);
    L1Serial.write(moveData.angSpeed.b, 4);
    L1Serial.write(lineTrack);
    L1Serial.write(lineAvoid);
    L1Serial.write(calibrate);
}

void readLayer1() {
    while (L1Serial.available() >= LAYER1_SEND_PACKET_SIZE) {
        uint8_t syncByte = L1Serial.read();
        if (syncByte == LAYER1_REC_SYNC_BYTE) {
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
    // will tell the robot if its supposed to attack or defend
    if (bt.isConnected) {
        // if role is undecided, pick default, else return the role that has
        // been previously assigned
        return Role::undecided ? defaultRole : role;
    } else if (bt.previouslyConnected) {
        // robot will come back in as goalie
        return Role::defend;
    } else {
        // if robot is only one on field, default to striker
        return Role::attack;
    }
}

bool shouldSwitchRoles(BluetoothData attackerData, BluetoothData defenderData) {
    // switch roles if goalie has ball or ball is much closer to goalie or
    // striker went out out field
    return 0;
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
        // Robot ID 1 (default defender) decides on role

        BluetoothData attackerData =
            role == Role::attack ? btData : bt.otherData;
        BluetoothData defenderData =
            role == Role::defend ? btData : bt.otherData;

        if (shouldSwitchRoles(attackerData, defenderData)) {
            role = role == Role::attack ? Role::defend : Role::attack;
        }
    } else {
        // Robot ID 0 is always the the opposite of the other robot
        role = bt.otherData.role == Role::attack ? Role::defend : Role::attack;
    }

    if (role != previousRole && previousRole == Role::attack) {
        // If switched to defender, move to the side to prevent collision
        movingSideways = true;
        Point sidewaysCoordinate = Point(40 * sign(botCoords.x), botCoords.y);
    } else {
        movingSideways = false;
    }
}

void updateBluetooth() {
    ballData.x = 60;
    ballData.y = 40;
    ballData.visible = true;
    botCoords.x = 250;
    botCoords.y = 0;
    role = Role::attack;
    onField = true;
    btData = BluetoothData(ballData, botCoords, role, onField);
    bt.update(btData);
    if (bt.isConnected && roleSwitching) {
        updateRole();
    } else if (bt.previouslyConnected) {
        role = Role::defend;
    }
}

void updateDebug() {
    ballData.angle = 0;
    ballData.dist = 0;
    bt.updateDebug(bbox, ballData);
}

void updateAllData() {
    readLayer1();
    heading = cmp.read();
    if (readTOF()) updatePosition();
    updateDebug();
    camera.update();
    updateBallData();
    // if (bluetoothTimer.timeHasPassed()) updateBluetooth();
}

#endif