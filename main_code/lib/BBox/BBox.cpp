#include "BBox.h"

void BBox::begin() {
    for (int i = 0; i < 4; i++) {
        tofAvg[i].begin();
    }
}

void BBox::update(TOFBuffer tof, LineData lineData, float heading) {
    // TODO: use kalman filter to detect if TOF is temporarily blocked?
    // TODO: integrate camera coordinates as weighted sum?
    // TODO: integrate light sensors to confirm x-position

    for (int i = 0; i < 4; i++) {
        // update moving average for each TOF
        tofAvg[i].reading(tof.vals[i]);
    }
    

    int frontTOF = tofAvg[0].getAvg();
    int leftTOF = tofAvg[1].getAvg();
    int backTOF = tofAvg[2].getAvg();
    int rightTOF = tofAvg[3].getAvg();

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
    Xconfidence = min(1, (float)180 / (float)width);
    Yconfidence = min(1, (float)180 / (float)height);


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
    String dir[4] = {"Front", "Left", "Back", "Right"};
    for (int i = 0; i < 4; i++) {
        Serial.print(dir[i] + ": ");
        Serial.print(tofAvg[i].getAvg());
        Serial.print("  ");
    }
    Serial.println();
}

void BBox::checkFieldDims() {
    String axis[2] = {"X-axis", "Y-axis"};
    for (int i = 0; i < 2; i++) {
        Serial.print(axis[i] + ": ");
        Serial.print(tofAvg[i].getAvg() + tofAvg[i * 2].getAvg());
        Serial.print("  ");
    }
    Serial.println();
}