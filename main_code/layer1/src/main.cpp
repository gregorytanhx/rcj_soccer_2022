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

float lastLineAngle;
MyTimer lineTimer(1000);

PID lineTrackPID(0.6, 0, 0.5);

bool lineTrack = true;
bool lineAvoid = false;
bool calibrate = false;
bool doneCalibrating = false;

uint8_t robotID;
int spd = 0;
int cnt = 0;
long lastRecvTime;

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
//#define SET_ID
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
    // pinMode(PB10, OUTPUT);
    // pinMode(PB11, OUTPUT);
    
}



void loop() {

    receiveData();
   
    calibrate = false;

    if (calibrate) {
        if (doneCalibrating) {
            light.read();
            light.sendVals();
        } else {
            light.calibrate();
            doneCalibrating = true;
        }

    } else {
        light.read();
        if (light.doneReading()) {
            
            //light.printLight();
            // light.printThresh();

            light.getLineData(lineData);
            // if (lineData.onLine) {
            //     L1DebugSerial.print("Line Angle: ");
            //     L1DebugSerial.print(lineData.lineAngle.val);
            //     L1DebugSerial.print("\tChord Length: ");
            //     L1DebugSerial.println(lineData.chordLength.val);
            // }
            sendData();
            
        }

        if (lineData.onLine) {
            if (lastLineAngle >= 0 &&
                abs(lastLineAngle - lineData.lineAngle.val) >= 90) {
                // prevent line angle from changing
                lineData.lineAngle.val = lastLineAngle;
                // allow chord length to keep increasing as robot
                // goes over centre of line
                lineData.chordLength.val = 2 - lineData.chordLength.val;
            }
            if (lineTrack) {
                // follow line
                lineTimer.update();
                float closestAngle = nonReflex(light.getClosestAngle(angle));
                // use PID to control angle of correction
                float correction = lineTrackPID.update(closestAngle - angle);
                float moveAngle = angle + correction;
                float dist = light.chordLength > 1 ? light.chordLength - 1
                                                   : 1 - light.chordLength;
                // L1DebugSerial.print("Reference angle: ");
                // L1DebugSerial.print(angle);
                // L1DebugSerial.print("Line track angle: ");
                // L1DebugSerial.println(closestAngle);

                // use chord length to adjust speed

                motors.setMove(speed, closestAngle, rotation, angSpeed);

            } else if (lineAvoid) {
                // avoid line by moving in opposite direction to line
                float moveAngle = fmod(lineData.lineAngle.val + 180, 360);
                // L1DebugSerial.print("Line Angle: ");
                //     L1DebugSerial.print(lineData.lineAngle.val);
                //     L1DebugSerial.print("\tChord Length: ");
                //     L1DebugSerial.println(lineData.chordLength.val);

                motors.setMove(fmax(60 * lineData.chordLength.val, 30),
                               moveAngle, rotation, angSpeed);
            } else {
                // ignore line
                motors.setMove(speed, angle, rotation, angSpeed);
            }

            lastLineAngle = lineData.lineAngle.val;
        } else {
            // no line detected, move according to teensy instructions
            motors.setMove(speed, angle, rotation, angSpeed);

            // reset last line angle
            lastLineAngle = -1;
        }

        motors.moveOut();
    }
}
