#include "BBox.h"

void BBox::begin() {
    for (int i = 0; i < 4; i++) {
        tofAvg[i].begin();
    }
}

void BBox::update(TOFBuffer &tof, LineData &lineData, float heading,
                  Camera &camera) {
    // TODO: use kalman filter to detect if TOF is temporarily blocked?
    // TODO: integrate camera coordinates as weighted sum?
    // TODO: integrate light sensors to confirm x-position
    flagCnt = 0;
    for (int i = 0; i < 4; i++) {
        // update moving average for each TOF
        //  tof.vals[i] = cos(deg2rad(heading)) * tof.vals[i];  // correct for
        //  angle
        tofVals[i] = tofAvg[i].reading(tof.vals[i]);
        if (tof.vals[i] < 150) {
            tofFlag[i] = 1;
            flagTimer[i] = millis();
            flagCnt++;
            prevTOF[i] = tof.vals[i];
        } else if (tof.vals[i] < prevTOF[i] * 0.7) {
            if (!tofFlag[i]) flagTimer[i] = millis();
            tofFlag[i] = 2;
        } else {
            tofFlag[i] = 0;
            prevTOF[i] = tof.vals[i];
        }
        if (millis() - flagTimer[i] > 2000) {
            tofFlag[i] = 0;
            prevTOF[i] = tof.vals[i];
        }
    }

    int frontTOF = tofVals[0];
    int rightTOF = tofVals[1];
    int backTOF = tofVals[2];
    int leftTOF = tofVals[3];

    // measured from left
    Xstart = FIELD_WIDTH / 2 - rightTOF;
    Xend = leftTOF - FIELD_WIDTH / 2;
    width = abs(Xend - Xstart);
    x = (Xstart + Xend) / 2;
    Ystart = FIELD_HEIGHT / 2 - frontTOF;
    Yend = backTOF - FIELD_HEIGHT / 2;

    // measured from top
    if (!tofFlag[0] && !tofFlag[2]) {
        height = abs(Yend - Ystart);
        y = (Ystart + Yend) / 2;
    } else if (!tofFlag[0])
        y = Ystart;
    else if (!tofFlag[2])
        y = Yend;
    else {
        if (tofFlag[0] == 2 && tofFlag[2] == 1)
            y = Ystart;
        else if (tofFlag[2] == 1 && tofFlag[0] == 2)
            y = Yend;
        else if (tofFlag[0] == 2 && tofFlag[2] == 2)
            y = (flagTimer[0] < flagTimer[2]) ? Yend : Ystart;
        else
            y = 0;
    }

    if (!tofFlag[1] && !tofFlag[3]) {
        height = abs(Xstart - Xend);
        x = (Xstart + Xend) / 2;
    } else if (!tofFlag[1])
        x = Xstart;
    else if (!tofFlag[3])
        x = Xend;
    else {
        if (tofFlag[1] == 2 && tofFlag[3] == 1)
            x = Xstart;
        else if (tofFlag[1] == 1 && tofFlag[3] == 2)
            x = Xend;
        else if (tofFlag[1] == 2 && tofFlag[3] == 2)
            x = (flagTimer[0] < flagTimer[2]) ? Xend : Xstart;
        else
            x = 0;
    }

    // take area of robot over area of bbox as confidence score
    Xconfidence = min(1, (float)200 / (float)width);
    Yconfidence = min(1, (float)200 / (float)height);

    // if (Xconfidence < 0.5) x = camera.centreVector.x;
    // if (Yconfidence < 0.5) y = camera.centreVector.y;

    // if (lineData.onLine) {
    //     float lineAngle = nonReflex(lineData.lineAngle.val + heading);
    //     if (angleIsInside(85, 95, lineAngle)) {
    //         // right edge of field
    //         x = FIELD_WIDTH / 2 - 250;
    //         Xconfidence = 1;
    //     } else if (angleIsInside(-85, -95, lineAngle)) {
    //         // left edge of field
    //         Xconfidence = 1;
    //         x = -FIELD_WIDTH /2 + 250;
    //     } else if (angleIsInside(-5, 5, lineAngle)) {
    //         // at the top corners
    //         Yconfidence = 1;
    //         y = FIELD_HEIGHT / 2 - 250;
    //     } else if (angleIsInside(-175, 175, lineAngle)) {
    //         // at the top corners
    //         Yconfidence = 1;
    //         y = -FIELD_HEIGHT / 2 + 250;
    //     }
    // }
}

void BBox::print() {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" X Confidence: ");
    Serial.print(Xconfidence);
    Serial.print(" Y Confidence: ");
    Serial.println(Yconfidence);
}

void BBox::printTOF() {
    String dir[4] = {"Front", "Right", "Back", "Left"};
    for (int i = 0; i < 4; i++) {
        Serial.print(dir[i] + ": ");
        Serial.print(tofVals[i]);
        Serial.print("  ");
    }
    Serial.println();
}

void BBox::checkFieldDims() {
    String axis[2] = {"Y-axis", "X-axis"};
    for (int i = 0; i < 2; i++) {
        Serial.print(axis[i] + ": ");
        Serial.print(tofVals[i] + tofVals[i + 2]);
        Serial.print("  ");
    }
    Serial.println();
}

int BBox::processTOFout(int tofThresh) {
    tofOutCnt = 0;

    for (int i = 0; i < 4; i++) {
        if (tofFlag[i] == 1) continue;
        if (tofVals[i] < tofThresh) {
            tofOut[tofOutCnt] = i;
            tofOutCnt++;
        }
    }

    if (tofOutCnt == 0 || tofOutCnt == 4)
        outAngle = -1;
    else if (tofOutCnt == 1)
        outAngle = mod(tofOut[0] * 90 + 180, 360);
    else if (tofOutCnt == 2) {
        if ((tofOut[0] == 0 && tofOut[1] == 2) ||
            (tofOut[0] == 1 && tofOut[1] == 3)) {
            outAngle = -1;  // ignore cases of front && back or left && right
        } else {
            if (tofOut[0] == 0 && tofOut[1] == 3) {
                tofOut[0] = 4;  // special case to make avg method work
            }
            outAngle =
                mod((float)(tofOut[0] + tofOut[1]) * 0.5 * 90 + 180, 360);
        }
    }
    if (tofOutCnt == 3) {
        if (tofOut[0] == 0) {
            if (tofOut[1] == 1) {
                if (tofOut[2] == 2) outAngle = 270;
                // detect front, right, back, move in 270 deg
                else
                    outAngle = 180;
                // detect front, right, left, move in 180 deg
            } else
                outAngle = 90;
            // detect front, left, back, move in 90 deg
        } else
            outAngle = 0;
        // detect right, back, left move in 0 deg
    }
    return outAngle;
}