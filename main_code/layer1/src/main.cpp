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

// buffer for receiving data from teensy
motorBuffer buffer;

float lastLineAngle;
MyTimer lineTimer(1000);

PID lineTrackPID(0.6, 0, 0.5);

bool lineTrack = true;
bool lineAvoid = false;
bool calibrate = false;
bool doneCalibrating = false;

uint8_t robotID;
int spd = 0;

void sendData() {
    L1CommSerial.write(LAYER1_SEND_SYNC_BYTE);
    L1CommSerial.write(lineData.lineAngle.b, sizeof(lineData.lineAngle.b));
    L1CommSerial.write(lineData.initialLineAngle.b,
                       sizeof(lineData.initialLineAngle.b));
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
    
#ifdef SET_ID
    robotID = ID;
    eeprom_buffered_write_byte(EEPROM_ID_ADDR, ID);
    eeprom_buffer_flush();
#else
    eeprom_buffer_fill();
    robotID = eeprom_buffered_read_byte(EEPROM_ID_ADDR);
#endif
    light.init();
   
    motors.init(robotID);

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
    receiveData();
    // motors.setMove(30, 0, 0);
    // motors.moveOut();
    
    if (calibrate) {
        if (doneCalibrating) {
            light.read();
            light.sendVals();
        } else {
            light.calibrate();
            doneCalibrating = true;
        }

    } else {
        light.readRaw();

        if (light.doneReading()) {
            light.printLight();
            // light.printThresh();
            light.getLineData(lineData);
            sendData();
            // if (lineData.onLine) {
            //     L1DebugSerial.print("Line Angle: ");
            //     L1DebugSerial.print(lineData.lineAngle.val);
            //     L1DebugSerial.print("\tChord Length: ");
            //     L1DebugSerial.println(lineData.chordLength.val);
            // }
        }

        if (lineData.onLine) {
            if (lineTrack) {
                angle = 0;
                // follow line
                lineTimer.update();
                float closestAngle = nonReflex(light.getClosestAngle(angle));
                // use PID to control angle of correction
                // float correction = lineTrackPID.update(closestAngle - angle);
                // float moveAngle = angle + correction;

                motors.setMove(closestAngle, moveAngle, 0);

            } else if (lineAvoid) {
                // avoid line by moving in opposite direction to line
                float moveAngle = fmod(lineData.lineAngle.val + 180, 360);

                motors.setMove(fmax(60 * lineData.chordLength.val, 30),
                               moveAngle, 0);
            } else {
                // ignore line
                motors.setMove(speed, angle, rotation);
            }
            if (lastLineAngle >= 0 &&
                abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
                // prevent line angle from changing
                lineData.lineAngle.val = lastLineAngle;
                // allow chord length to keep increasing as robot
                // goes over centre of line
                lineData.chordLength.val = 2 - lineData.chordLength.val;
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
