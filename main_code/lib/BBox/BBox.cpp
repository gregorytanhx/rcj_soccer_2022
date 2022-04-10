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

    // measured from top
    Ystart = FIELD_HEIGHT / 2 - frontTOF;
    Yend = backTOF - FIELD_HEIGHT / 2;

    width = abs(Xend - Xstart);
    height = abs(Yend - Ystart);

    x = (Xstart + Xend) / 2;
    y = (Ystart + Yend) / 2;

    // take area of robot over area of bbox as confidence score
    Xconfidence = min(1, (float)200 / (float)width);
    Yconfidence = min(1, (float)200 / (float)height);

    // if (Xconfidence < 0.5) x = camera.centreVector.x;
    // if (Yconfidence < 0.5) y = camera.centreVector.y;

    if (lineData.onLine) {
        float lineAngle = nonReflex(lineData.lineAngle.val + heading);
        if (angleIsInside(85, 95, lineAngle)) {
            // right edge of field
            x = FIELD_WIDTH - 250;
            Xconfidence = 1;
        } else if (angleIsInside(-85, -95, lineAngle)) {
            // left edge of field
            Xconfidence = 1;
            x = -FIELD_WIDTH + 250;
        } else if (angleIsInside(-5, 5, lineAngle)) {
            // at the top corners
            Yconfidence = 1;
            y = FIELD_HEIGHT / 2 - 250;
        } else if (angleIsInside(-175, 175, lineAngle)) {
            // at the top corners
            Yconfidence = 1;
            y = -FIELD_HEIGHT / 2 + 250;
        }
    }
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

int BBox::TOFout() {
    tofOutCnt = 0;

    for (int i = 0; i < 4; i++) {
        if (tofFlag[i]) continue;
        if (tofVals[i] < 400) {
            tofOut[tofOutCnt] = i;
            tofOutCnt++;
        }
    }

    if (tofOutCnt == 0 || tofOutCnt == 4) return -1;
    if (tofOutCnt == 1)  return mod(tofOut[0] * 90 + 180, 360);
    if (tofOutCnt == 2) {
        if ((tofOut[0] == 0 && tofOut[1] == 2) ||
            (tofOut[0] == 1 && tofOut[1] == 3)) {
            return -1;  // ignore cases of front && back or left && right
        }
        if (tofOut[0] == 0 && tofOut[1] == 3)
            tofOut[0] = 4;  // special case to make avg method work
        return mod((float)(tofOut[0] + tofOut[1]) * 0.5 * 90 + 180, 360);
    }
    if (tofOutCnt == 3) {
        if (tofOut[0] == 0) {
            if (tofOut[1] == 1) {
                if (tofOut[2] == 2)
                    return 270;  // detect front, right, back, move in 270 deg
                else
                    return 180;  // detect front, right, left, move in 180 deg
            } else
                return 90;  // detect front, left, back, move in 90 deg
        } else
            return 0;  // detect right, back, left move in 0 deg
    }
}