#include <Arduino.h>
#include <Common.h>
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
float angSpeed = -1.0;
LineData lineData;

typedef union motorBuffer {
    float vals[4];
    uint8_t b[sizeof(vals)];
} motorBuffer;

// buffer for receiving data from teensy
motorBuffer buffer;

float lastLineAngle, lastChordLength;
MyTimer lineTimer(1000);

PID lineTrackPID(0.6, 0, 0.5);

bool lineTrack = true;
bool lineAvoid = false;
bool calibrate = false;
bool doneCalibrating = false;
bool prevLine;

uint8_t robotID;
int spd = 0;
int cnt = 0;
long lastRecvTime;
long lastOutTime, lastInTime;

void sendData() {
    L1CommSerial.write(LAYER1_SEND_SYNC_BYTE);
    L1CommSerial.write(lineData.lineAngle.b, 4);
    L1CommSerial.write(lineData.initialLineAngle.b, 4);
    L1CommSerial.write(lineData.chordLength.b, 4);
    L1CommSerial.write(lineData.onLine);
    L1CommSerial.flush();
}

void receiveData() {
    // receive teensy data
    while (L1CommSerial.available() >= 20) {
        // L1DebugSerial.println(micros() - lastRecvTime);
        // lastRecvTime = micros();
        uint8_t syncByte = L1CommSerial.read();
        if (syncByte == LAYER1_SYNC_BYTE_START) {
            // exclude last 3 bytes
            for (int i = 0; i < 16; i++) {
                buffer.b[i] = L1CommSerial.read();
            }
            uint8_t tmp1 = (bool)L1CommSerial.read();
            uint8_t tmp2 = (bool)L1CommSerial.read();
            uint8_t syncByte2 = L1CommSerial.read();
            if (syncByte2 == LAYER1_SYNC_BYTE_END) {
                speed = buffer.vals[0];
                angle = buffer.vals[1];
                rotation = buffer.vals[2];
                angSpeed = buffer.vals[3];
                lineTrack = tmp1;
                lineAvoid = tmp2;
            }
        }
    }

    // L1DebugSerial.print(speed);
    // L1DebugSerial.print(" ");
    // L1DebugSerial.print(angle);
    // L1DebugSerial.print(" ");
    // L1DebugSerial.print(rotation);
    // L1DebugSerial.print(" ");
    // L1DebugSerial.print(angSpeed);
    // L1DebugSerial.print(" ");
    //     L1DebugSerial.print(lineTrack);
    // L1DebugSerial.print(" ");
    // L1DebugSerial.print(lineAvoid);
    // L1DebugSerial.print(" ");
    // L1DebugSerial.println();
}
int maxVals[32];

// handle line avoidance directly through stm32
// #define SET_ID
void setup() {
#ifdef SET_ID
    robotID = ID;
    eeprom_buffered_write_byte(0, 0);
    eeprom_buffer_flush();
#else
    eeprom_buffer_fill();
    robotID = eeprom_buffered_read_byte(0);
    L1DebugSerial.println(robotID);
#endif
    light.begin(robotID);
    motors.begin(robotID);

    L1DebugSerial.begin(9600);
    L1CommSerial.begin(STM32_BAUD);
    // lineTimer.update();
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, HIGH);
   
}

void loop() {
    receiveData();

    calibrate = false;
    // lineTrack = true;
    // lineAvoid = true;

    if (calibrate) {
        if (doneCalibrating) {
            light.read();
            
        } else {
            light.calibrate();
            doneCalibrating = true;
        }

    } else {
        light.read();
        if (light.doneReading()) {
            // light.printLight();
            light.getLineData(lineData);
            if (lineData.onLine) {
                lastOutTime = millis();
                if (lastLineAngle >= 0 &&
                    abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
                    // prevent line angle from changing
                    lineData.lineAngle.val = lastLineAngle;
                    // allow chord length to keep increasing as robot
                    // goes over centre of line
                    lineData.chordLength.val = 2 - lineData.chordLength.val;
                }
            } else {
                if (prevLine && lastChordLength > 1 &&
                    (millis() - lastOutTime < 100) && (millis() - lastInTime > 300)) {
                    // previously on line, now out of field
                    lineData.onLine = true;
                    lineData.chordLength.val = 2;
                    lineData.lineAngle.val = lastLineAngle;
                }
            }
            
        }

       

        if (lineData.onLine) {
           
            // L1DebugSerial.print("Line Angle: ");
            // L1DebugSerial.print(lineData.lineAngle.val);
            // L1DebugSerial.print("\tChord Length: ");
            // L1DebugSerial.println(lineData.chordLength.val);

            if (lineTrack) {
                // follow line
                lineTimer.update();
                float closestAngle = nonReflex(light.getClosestAngle(angle));
                // use PID to control angle of correction
                // float correction = lineTrackPID.update(closestAngle - angle);
                // float moveAngle = angle + correction;
                float dist = light.chordLength > 1 ? light.chordLength - 1
                                                   : 1 - light.chordLength;

                // use chord length to adjust speed
                motors.setMove(speed * 0.9 + 30 * dist, closestAngle, rotation, angSpeed);

            } else if (lineAvoid) {
                // avoid line by moving in opposite direction to line
                float moveAngle = fmod(lineData.lineAngle.val + 180, 360);
                // L1DebugSerial.println("Line Avoid");
                motors.setMove(40 + 50 * lineData.chordLength.val,
                               moveAngle, rotation, angSpeed);
            } else {
                // ignore line
                motors.setMove(speed, angle, rotation, angSpeed);
            }

            lastLineAngle = lineData.lineAngle.val;
            lastChordLength = lineData.chordLength.val;

        } else {
            lastInTime = millis();
            

            // no line detected, move according to teensy instructions
            motors.setMove(speed, angle, rotation, angSpeed);
            if (millis() - lastOutTime > 1000) {
                // reset last line angle
                lastLineAngle = -1;
                lastChordLength = 0;
            }
        }
        prevLine = lineData.onLine;
        if (light.doneReading()) sendData();
        motors.moveOut();
    }

}
