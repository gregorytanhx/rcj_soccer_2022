#include "Light.h"

Light::Light() {
    sigA = SIG_A;
    sigB = SIG_B;

    pinsA[0] = mux_A1;
    pinsA[1] = mux_A2;
    pinsA[2] = mux_A3;
    pinsA[3] = mux_A4;

    pinsB[0] = mux_B1;
    pinsB[1] = mux_B2;
    pinsB[2] = mux_B3;
    pinsB[3] = mux_B4;
}

void Light::init() {
    pinMode(mux_A1, OUTPUT);
    pinMode(mux_A2, OUTPUT);
    pinMode(mux_A3, OUTPUT);
    pinMode(mux_A4, OUTPUT);

    pinMode(mux_B1, OUTPUT);
    pinMode(mux_B2, OUTPUT);
    pinMode(mux_B3, OUTPUT);
    pinMode(mux_B4, OUTPUT);

    for (int i = 0; i < 4; i++) {
        digitalWrite(pinsA[i], LOW);
        digitalWrite(pinsB[i], LOW);
    }

    // retrieve threshold from eeprom memory
#ifdef USE_EEPROM
    eeprom_buffer_fill();

    for (auto i : lightMap) {
        lightThresh.b[i * 2] = eeprom_buffered_read_byte(i + 1);
        lightThresh.b[i * 2 + 1] = eeprom_buffered_read_byte(i + 33);
    }
#else
    for (int i = 0; i < 32; i++) {
        lightThresh.vals[i] = fixedThresh[i];
    }
#endif

    readTimer = micros();
}
void Light::printThresh() {
    L1DebugSerial.print("Thresh: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(lightThresh.vals[i]);
        L1DebugSerial.print(", ");
    }
    L1DebugSerial.println();
}

void Light::printLight() {
    L1DebugSerial.print("Values: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(lightVals[i]);
        L1DebugSerial.print(", ");
    }
    L1DebugSerial.println();
}

int Light::readMux(int channel, int controlPin[4], int sig) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }
    // read the value at the SIG pin

    int val = analogRead(sig);
    // return the value
    return val;
}

void Light::read() {
    // non blocking light read
    if (micros() - readTimer >= MUX_DELAY) {
        bool out = false;

        int idx = lightMap[lightCnt];
        lightVals[idx] = readMux(lightCnt, pinsA, sigA);
        if (lightVals[idx] > lightThresh.vals[idx]) {
            lineDetected[outSensors] = idx;
            outSensors++;
        }

        idx = lightMap[lightCnt + 16];
        lightVals[idx] = readMux(lightCnt, pinsB, sigB);
        if (lightVals[idx] > lightThresh.vals[idx]) {
            lineDetected[outSensors] = idx;
            outSensors++;
        }

        lightCnt++;
        lightCnt %= 16;
        readTimer = micros();
        // delayMicroseconds(100);
    }
}

bool Light::doneReading() { return lightCnt == 0; }

void Light::sendVals() {
    L1CommSerial.write(LAYER1_SEND_SYNC_BYTE);
    for (int i = 0; i < 32; i++) {
        lightBuffer.vals[i] = lightVals[i];
        L1CommSerial.write(lightBuffer.b, 2);
    }
}

void Light::calibrate() {
#ifdef USE_EEPROM
#ifdef DEBUG
    L1DebugSerial.print("Calibrating...");
#endif
    // reset max and min values each time
    for (int i = 0; i < 32; i++) {
        maxVals[i] = 0;
        minVals[i] = 1200;
    }
    const unsigned long timeOut = 60000;
    unsigned long timeStart = millis();
    while ((millis() - timeStart) < timeOut) {
        read();
        if (doneReading()) {
            for (auto i : lightMap) {
                if (lightVals[i] > maxVals[i]) {
                    maxVals[i] = lightVals[i];
                }
                if (lightVals[i] < minVals[i]) {
                    minVals[i] = lightVals[i];
                }

                lightThresh.vals[i] = (maxVals[i] + minVals[i]) / 2;
            }
        }

        // printLight();
    }
#ifdef DEBUG
    L1DebugSerial.print("Done!");
#endif

#endif

#ifdef DEBUG
    L1DebugSerial.print("Max: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(maxVals[lightMap[i]]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
    L1DebugSerial.print("Min: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(minVals[lightMap[i]]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
    L1DebugSerial.print("Thresh: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(lightThresh.vals[lightMap[i]]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
#endif

#ifdef USE_EEPROM
    // write calibrated threshold to eeprom memory
    for (auto i : lightMap) {
        eeprom_buffered_write_byte(i + 1, lightThresh.b[i * 2]);
        eeprom_buffered_write_byte(i + 33, lightThresh.b[i * 2 + 1]);
    }
    eeprom_buffer_flush();

#endif

    L1CommSerial.write(LAYER1_SEND_SYNC_BYTE);
    // send thresholds to teensy for checking
    for (int i = 0; i < 32; i++) {
        L1CommSerial.write(lightThresh.b, 2);
    }
}

// TO DO: WRITE CALIBRATED LIGHT VALS TO STM32 EEPROM MEMORY

void Light::getLineData(LineData& data) {
    float vecX = 0;
    float vecY = 0;
    int chordStart = 0;
    int chordEnd = 0;
    float largestDiff = 0;
    onLine = outSensors > 0;
    if (onLine) {
        // get line angle and chord length
        for (int i = 0; i < outSensors; i++) {
#ifdef DEBUG
            // L1DebugSerial.print(lineDetected[i]);
            // L1DebugSerial.print(" Val: ");
            // L1DebugSerial.print(lightVals[lineDetected[i]]);
            // L1DebugSerial.print(" Thresh: ");
            // L1DebugSerial.print(lightThresh.vals[lineDetected[i]]);
            // L1DebugSerial.print(" ");

#endif
            float tmpAngle = deg2rad(lineDetected[i] * 360 / 32);
            vecY += cos(tmpAngle);
            vecX += sin(tmpAngle);

            for (int j = i + 1; j < outSensors; j++) {
                float tmpDiff = angleDiff(lineDetected[i] * 360 / 32,
                                          lineDetected[j] * 360 / 32);
                if (tmpDiff > largestDiff) {
                    chordStart = lineDetected[j];
                    chordEnd = lineDetected[i];
                    largestDiff = tmpDiff;
                }
            }
        }
        // L1DebugSerial.print(chordStart);
        // L1DebugSerial.print(" ");
        // L1DebugSerial.println(chordEnd);
        chordLength = norm(abs(chordStart - chordEnd), 16, 1);
        lineAngle = rad2deg(atan2(vecX, vecY));
        if (lineAngle < 0) lineAngle += 360;
        // lineAngle = fmod(lineAngle + 180, 360);
    }
    outSensors = 0;

    // update data
    data.onLine = onLine;
    data.lineAngle.val = lineAngle;
    data.chordLength.val = chordLength;
}

float Light::getClosestAngle(float target) {
    float closestAngle = 0;
    float minDiff = 360;
    for (int i = 0; i < 32; i++) {
        if (lightVals[i]) {
            float tmpAngle = deg2rad(i * 360 / 32);
            float diff = angleDiff(tmpAngle, target);
            if (diff < minDiff) {
                minDiff = diff;
                closestAngle = tmpAngle;
            }
        }
    }
    return closestAngle;
}