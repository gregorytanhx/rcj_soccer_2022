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

    for (int i = 0; i < 32; i++) {
        maxVals[i] = 0;
        minVals[i] = 1200;
    }
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
    #ifdef DEBUG
        L1DebugSerial.println("Loading thresholds...");
    #endif
    for (int i = 0; i < 32; i++) {
        lightThresh.b[i*2] = eeprom_buffered_read_byte(i+1);
        lightThresh.b[i*2+1] = eeprom_buffered_read_byte(i+33);
    
    #ifdef DEBUG
        L1DebugSerial.print(i);
        L1DebugSerial.print("Thresh: ");
        L1DebugSerial.println(lightThresh.vals[i]);
    #endif
    }
#endif
    
}

int Light::readMux(int channel, int controlPin[4], int sig) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }
    // read the value at the SIG pin
    delayMicroseconds(5);
    int val = analogRead(sig);
    // return the value
    return val;
}

void Light::read() {
    bool out = false;
    outSensors = 0;
    for (int i = 0; i < 16; i++) {
        lightVals[i] = readMux(i, pinsB, sigB);
        if (lightVals[i] > lightThresh.vals[i]) {
            lineDetected[outSensors] = i;
            outSensors++;
        }
    }

    for (int i = 16; i < 32; i++) {
        lightVals[i] = readMux(i - 16, pinsA, sigA);
        if (lightVals[i] > lightThresh.vals[i]) {
            lineDetected[outSensors] = i;
            outSensors++;
        }
    }
    
    onLine = outSensors > 0;


// #ifdef DEBUG
//     for (int i = 0; i < 32; i++) {
//         L1DebugSerial.print(lightVals[i]);
//         L1DebugSerial.print(",");
//     }    
//     L1DebugSerial.println();
// #endif
   
}

void Light::calibrate() {
    
    
    lightTimer.update();

#ifdef USE_EEPROM 
    while (!lightTimer.timeHasPassed(false)) {
        read();
        for (int i = 0; i < 32; i++) {
            if (lightVals[i] > maxVals[i]) 
                maxVals[i] = lightVals[i];

            if (lightVals[i] < minVals[i]) 
                minVals[i] = lightVals[i];
            
            lightThresh.vals[i] = (maxVals[i] + minVals[i]) / 2;

        }
    }
#else
    read();
    for (int i = 0; i < 32; i++) {
        if (lightVals[i] > maxVals[i]) 
            maxVals[i] = lightVals[i];

        if (lightVals[i] < minVals[i]) 
            minVals[i] = lightVals[i];
        
        lightThresh.vals[i] = (maxVals[i] + minVals[i]) / 2;

    }
    delay(100);
#endif

#ifdef DEBUG
    L1DebugSerial.print("Max: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(maxVals[i]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
    L1DebugSerial.print("Min: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(minVals[i]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
    L1DebugSerial.print("Thresh: ");
    for (int i = 0; i < 32; i++) {
        L1DebugSerial.print(lightThresh.vals[i]);
        L1DebugSerial.print(" ");
    }
    L1DebugSerial.println();
#endif

#ifdef USE_EEPROM
    // write calibrated threshold to eeprom memory
    for (int i = 0; i < 32; i++) {
        eeprom_buffered_write_byte(i + 1, lightThresh.b[i*2]);
        eeprom_buffered_write_byte(i + 33, lightThresh.b[i*2+1]);
    }
    eeprom_buffer_flush();
#endif
}

// TO DO: WRITE CALIBRATED LIGHT VALS TO STM32 EEPROM MEMORY

void Light::getLineData(LineData& data) {
    float vecX = 0;
    float vecY = 0;
    int chordStart = 0;
    int chordEnd = 0;
    float largestDiff = 0;

    if (onLine) {
        // get line angle and chord length
        for (int i = 0; i < outSensors; i++) {
            float tmpAngle = deg2rad(i * 360 / 32);
            vecY += cos(tmpAngle);
            vecX += sin(tmpAngle);

            for (int j = i + 1; j < outSensors; j++) {
                float tmpDiff = angleDiff(i * 360 / 32, j * 360 / 32);
                if (tmpDiff > largestDiff) {
                    chordStart = j;
                    chordEnd = i;
                    largestDiff = tmpDiff;
                }
            }
        }
        chordLength = norm(abs(chordStart - chordEnd), 15, 1);
        lineAngle = rad2deg(atan2(vecX, vecY));
        if (lineAngle < 0) lineAngle += 360;
        lineAngle = fmod(lineAngle + 180, 360);
    } 

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