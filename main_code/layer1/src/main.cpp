#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Light.h>
#include <Motor.h>
#include <PID.h>
#include <Pins.h>
#include <SoftwareSerial.h>
#include <MyTimer.h>

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
            // exclude last 2 bytes
            for (int i = 0; i < LAYER1_REC_PACKET_SIZE - 3; i++) {
                buffer.b[i] = L1CommSerial.read();
            }
            lineTrack = (bool) L1CommSerial.read();
            lineAvoid = (bool) L1CommSerial.read();
            calibrate = (bool) L1CommSerial.read();
        }
    }
    speed = buffer.vals[0];
    angle = buffer.vals[1];
    rotation = buffer.vals[2];
}



// handle line avoidance directly through stm32
void setup() {
    light.init();
    motors.init();


    L1DebugSerial.begin(9600);
    L1CommSerial.begin(STM32_BAUD);
    lineTimer.update();
    pinMode(LAYER1_LED, OUTPUT);
    digitalWrite(LAYER1_LED, HIGH);
}

void loop() {

    // light.read();
    // light.calibrate();
    receiveData();
    if (calibrate) {
        light.calibrate();
    } else {
        motors.setMove(speed, angle, rotation);
        motors.moveOut();
        //     L1DebugSerial.println("test");
        //     light.read();
        //     light.getLineData(lineData);
        // receiveData();
        // sendData();

        //     if (lineData.onLine) {

        // #ifdef DEBUG
        //         L1DebugSerial.print("Line Angle: ");
        //         L1DebugSerial.print(lineData.lineAngle.val);
        //         L1DebugSerial.println("\tChord Length: ");
        // #endif

        //         if (lineTrack) {
        //             lineTimer.update();
        //             float angle =
        //             nonReflex(light.getClosestAngle(moveData.angle.val));
        //             // use PID to control speed of correction
        //             float correction = lineTrackPID.update(angle -
        //             moveData.angle.val);

        //             motors.setMove(LINE_TRACK_SPEED + correction, angle, 0);
        //         } else if (lineAvoid) {
        //             if (abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
        //                 // allow chord length to keep increasing as robot
        //                 goes
        //                 // over centre of line
        //                 lineData.chordLength.val = 2 -
        //                 lineData.chordLength.val;
        //             }
        //             // line avoidance
        //             motors.setMove(speed * lineData.chordLength.val,
        //             lineData.lineAngle.val, 0);
        //         } else {
        //             motors.setMove(speed, angle, rotation);
        //         }
        //     }
        //     else {
        //         motors.setMove(speed, angle, rotation);
        //     }
        //     if (lineTimer.timeHasPassed()) {
        //         lineTrackPID.resetIntegral();
        //     }

        //     lastLineAngle = lineData.lineAngle.val;
    }
   
}