#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Light.h>
#include <Motor.h>
#include <MyTimer.h>
#include <PID.h>
#include <Pins.h>
#include <SoftwareSerial.h>

Light light;
Motors motors;
float speed = 0;
float angle = 0;
float rotation = 0;
LineData lineData;
MoveData moveData(0, 0, 0);

// buffer for receiving data from teensy
motorBuffer buffer;

float lastLineAngle;
MyTimer lineTimer(1000);

PID lineTrackPID(LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD);
bool lineTrack = false;
bool lineAvoid = true;
bool calibrate = false;
bool doneCalibrating = false;

int spd = 0;

void sendData() {
    L1CommSerial.write(LAYER1_SEND_SYNC_BYTE);
    L1CommSerial.write(lineData.lineAngle.b, sizeof(lineData.lineAngle.b));
    L1CommSerial.write(lineData.chordLength.b, sizeof(lineData.chordLength.b));
    L1CommSerial.write(lineData.onLine);
}

void receiveData() {
    // receive teensy data
    while (L1CommSerial.available() >= LAYER1_REC_PACKET_SIZE) {
        uint8_t syncByte = L1CommSerial.read();
        if (syncByte == LAYER1_REC_SYNC_BYTE) {
            // exclude last 3 bytes
            for (int i = 0; i < LAYER1_REC_PACKET_SIZE - 4; i++) {
                buffer.b[i] = L1CommSerial.read();
            }
            lineTrack = (bool)L1CommSerial.read();
            lineAvoid = (bool)L1CommSerial.read();
            calibrate = (bool)L1CommSerial.read();
            speed = buffer.vals[0];
            angle = buffer.vals[1];
            rotation = buffer.vals[2];
        }
    }
}
int maxVals[32];

// handle line avoidance directly through stm32
void setup() {
    light.init();

    motors.init();

    L1DebugSerial.begin(9600);
    L1CommSerial.begin(STM32_BAUD);
    // lineTimer.update();
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, HIGH);
    for (int i = 0; i < 32; i++) {
        maxVals[i] = 0;
    }
}

void loop() {
    // receiveData();
    // motors.setMove(speed, angle, rotation);
    // motors.moveOut();

    // motors.setMove(50, 180, 0);
    // motors.moveOut();
    // delay(1000);

    if (calibrate) {
        if (doneCalibrating) {
            light.read();
            light.sendVals();
        } else {
            light.calibrate();
            doneCalibrating = true;
        }

    } else {
        sendData();
        // L1DebugSerial.println(light.readMux(12, light.pinsA, light.sigA));
        // delayMicroseconds((100));
        // L1DebugSerial.println(light.readMux(11, light.pinsA, light.sigA));
        light.readRaw();

        if (light.doneReading()) {
            
            // for (int i = 0; i < 16; i++) {
            //     L1DebugSerial.print(light.lightVals[i]);
            //     L1DebugSerial.print(" ");
            // }
           
            for (int i = 16; i < 32; i++) {
                L1DebugSerial.print(light.lightVals[i]);
                L1DebugSerial.print(" ");
            }

            L1DebugSerial.println();

            //light.printLight();
            //light.printThresh();
            //light.getLineData(lineData);
            // if (lineData.onLine) {
            //     L1DebugSerial.print("Line Angle: ");
            //     L1DebugSerial.print(lineData.lineAngle.val);
            //     L1DebugSerial.print("\tChord Length: ");
            //     L1DebugSerial.println(lineData.chordLength.val);
            // }
        }

        if (lineData.onLine) {
            if (lineTrack) {
                // follow line
                lineTimer.update();
                float moveAngle =
                    nonReflex(light.getClosestAngle(moveData.angle.val));
                // use PID to control speed of correction
                float correction = 0;
                // lineTrackPID.update(angle - moveData.angle.val);

                motors.setMove(LINE_TRACK_SPEED + correction, moveAngle, 0);

            } else if (lineAvoid) {
                // avoid line
                if (lastLineAngle >= 0 &&
                    abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
                    // prevent line angle from changing too much
                    lineData.lineAngle.val = lastLineAngle;
                    // allow chord length to keep increasing as robot
                    // goes over centre of line
                    lineData.chordLength.val = 2 - lineData.chordLength.val;
                }

                // move in opposite direction to line
                float moveAngle = fmod(lineData.lineAngle.val + 180, 360);

                motors.setMove(fmin(60 * lineData.chordLength.val, 20), moveAngle, 0);
            } else {
                // ignore line
                motors.setMove(speed, angle, rotation);
            }

            lastLineAngle = lineData.lineAngle.val;

        } else {
            // no line detected, move according to teensy instructions
            motors.setMove(speed, angle, rotation);
            // motors.setMove(0, 0, 0);
            // reset last line angle
            lastLineAngle = -1;
        }
        if (lineTimer.timeHasPassed()) {
            lineTrackPID.resetIntegral();
        }

        motors.moveOut();
    }
}
