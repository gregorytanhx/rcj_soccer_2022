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

void Motors::setMove(float speed, float angle, float rotation) {
    float outSpeed = speed;

    a = sin(deg2rad(50 + angle)) * MOTOR_MULT;
    b = sin(deg2rad(50 - angle)) * MOTOR_MULT;

    fl = a - rotation * 0.1;
    fr = b + rotation * 0.1;
    bl = b - rotation * 0.1;
    br = a + rotation * 0.1;

    float factor = outSpeed / max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
    FL_OUT = round(fl * factor);
    FR_OUT = round(fr * factor);
    BL_OUT = round(bl * factor);
    BR_OUT = round(br * factor);

}

void Motors::moveOut() {
    analogWrite(FL_PWM, abs(FL_OUT));
    digitalWrite(FL_DIG, FL_OUT > 0 ? LOW : HIGH);

    analogWrite(FR_PWM, abs(FR_OUT));
    digitalWrite(FR_DIG, FR_OUT > 0 ? LOW : HIGH);

    analogWrite(BL_PWM, abs(BL_OUT));
    digitalWrite(BL_DIG, BL_OUT > 0 ? LOW : HIGH);

    analogWrite(BR_PWM, abs(BR_OUT));
    digitalWrite(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
}
