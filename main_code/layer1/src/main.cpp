#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Light.h>
#include <Motors.h>
#include <PID.h>
#include <Pins.h>

Light light;
Motors motors;
float speed;
float angle;
float rotation;
LineData lineData;
motorBuffer buffer;
float lastLineAngle;
Timer lineTimer(1000);

PID lineTrackPID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);
bool lineTrack = false;
bool lineAvoid = true;

void sendData() {
    Serial1.write(LAYER1_SEND_SYNC_BYTE);
    Serial1.write(lineData.lineAngle.b, sizeof(lineData.lineAngle.b));
    Serial1.write(lineData.chordLength.b, sizeof(lineData.chordLength.b));
    Serial1.write(lineData.onLine);
}

void receiveData() {
    // receive teensy data
    while (Serial1.available() >= LAYER1_REC_PACKET_SIZE) {
        uint8_t syncByte = Serial1.read();
        if (syncByte == LAYER1_REC_SYNC_BYTE) {
            for (int i = 0; i < MOTOR_PACKET_SIZE - 2; i++) {
                motorBuffer.b[i] = Serial1.read();
            }
            lineTrack = (bool)Serial1.read();
            lineAvoid = (bool)Serial1.read();
        }
    }
    speed = motorBuffer.vals[0];
    angle = motorBuffer.vals[1];
    rotation = motorBuffer.vals[2];
}

// handle line avoidance directly through stm32
void setup() {
    light.init();
    motors.init() pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
    L1DebugSerial.begin(9600);
    L1CommSerial.begin(STM32_BAUD);
    lineTimer.update();
}

void loop() {
    light.read();
    light.getLineData(LineData);
    receiveData();
    sendData();


    if (lineData.onLine) {

#ifdef DEBUG
        L1DebugSerial.print("Line Angle: ");
        L1DebugSerial.print(lineAngle);
        L1DebugSerial.println("\tChord Length: ");
#endif

        if (lineTrack) {
            lineTimer.update();
            float angle = nonReflex(light.getClosestAngle(moveData.angle));
            // use PID to control speed of correction
            float correction = lineTrackPID.update(angle - moveData.angle);

            motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
        } else if (lineAvoid) {
            if (abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
                // allow chord length to keep increasing as robot goes over
                // centre of line
                lineData.chordLength.val = 2 - lineData.chordLength.val;
            }
            // line avoidance
            motors.setMove(speed * lineData.chordLength.val, lineData.lineAngle.val, 0);
        } else {
            motors.setMove(speed, angle, rotation);
        }
    }
    else {
        motors.setMove(speed, angle, rotation);
    }
    if (lineTimer.timeHasPassed()) {
        lineTrackPID.resetIntegral();
    }

    lastLineAngle = lineData.lineAngle;
    motors.moveOut();
}