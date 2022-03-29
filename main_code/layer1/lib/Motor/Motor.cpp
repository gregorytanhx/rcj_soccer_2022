#include "Motor.h"

void Motors::init() {
    pinMode(FL_DIG, OUTPUT);
    pinMode(FL_PWM, OUTPUT);

    pinMode(FR_DIG, OUTPUT);
    pinMode(FR_PWM, OUTPUT);

    pinMode(BL_DIG, OUTPUT);
    pinMode(BL_PWM, OUTPUT);

    pinMode(BR_DIG, OUTPUT);
    pinMode(BR_PWM, OUTPUT);
}

void Motors::setMove(float speed, float angle, float rotation,
                     float angSpeed = -1.0) {
   
    // angle between motors is 100 from left to right, 80 from top to bottom
    if (angSpeed == -1.0) angSpeed = speed;

    x_co = sinf(deg2rad(angle)) * 0.6527036446661;
    y_co = cosf(deg2rad(angle)) * 0.7778619134302;

    fl = (x_co + y_co) * speed + (0.1 * rotation) * angSpeed;
    bl = (-x_co + y_co) * speed + (0.1 * rotation) * angSpeed;
    fr = (-x_co + y_co) * speed - (0.1 * rotation) * angSpeed;
    br = (x_co + y_co) * speed - (0.1 * rotation) * angSpeed;

    FL_OUT = round(fl);
    FR_OUT = round(fr);
    BL_OUT = round(bl);
    BR_OUT = round(br);
}

void Motors::moveOut() {
    analogWrite(FL_PWM, abs(FL_OUT));
    digitalWrite(FL_DIG, FL_OUT > 0 ? HIGH : LOW);

    analogWrite(FR_PWM, abs(FR_OUT));
    digitalWrite(FR_DIG, FR_OUT > 0 ? HIGH : LOW);

    analogWrite(BL_PWM, abs(BL_OUT));
    digitalWrite(BL_DIG, BL_OUT > 0 ? LOW : HIGH);

    analogWrite(BR_PWM, abs(BR_OUT));
    digitalWrite(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
}
