#include "BBox.h"

void BBox::update(TOFBuffer tof, LineData lineData, float heading) {
    // TODO: use kalman filter to detect if TOF is temporarily blocked?
    // TODO: integrate camera coordinates as weighted sum?
    // TODO: integrate light sensors to confirm x-position

    int frontTOF = tof.vals[0];
    int rightTOF = tof.vals[1];
    int backTOF = tof.vals[2];
    int leftTOF = tof.vals[3];

    Xstart = leftTOF;
    Xend = FIELD_WIDTH - rightTOF;
    Ystart = frontTOF;
    Yend = FIELD_HEIGHT - backTOF;

    width = Xend - Xstart;
    height = Yend - Ystart;

    // recenter to obtain coordinates
    Xstart -= FIELD_WIDTH / 2;
    Xend -= FIELD_WIDTH / 2;
    Ystart = FIELD_HEIGHT / 2 - Ystart;
    Yend = FIELD_HEIGHT / 2 - Yend;

    x = (Xstart - Xend) / 2;
    y = (Ystart - Yend) / 2;

    // take area of robot over area of bbox as confidence score
    Xconfidence = 180 / width;
    Yconfidence = 180 / height;

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