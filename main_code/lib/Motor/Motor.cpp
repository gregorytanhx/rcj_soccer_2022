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
    float outSpeed = map(speed, 0, 100, 0, MAX_PWM);

    float a = sin(deg2rad(50 + angle)) * MOTOR_MULT;
    float b = sin(deg2rad(50 - angle)) * MOTOR_MULT;

    float fl = a - rotation;
    float fr = b + rotation;
    float bl = b - rotation;
    float br = a + rotation;

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
    digitalWrite(FR_DIG, FR_OUT > 0 ? HIGH : LOW);

    analogWrite(BL_PWM, abs(BL_OUT));
    digitalWrite(BL_DIG, BL_OUT > 0 ? HIGH : LOW);

    analogWrite(BR_PWM, abs(BR_OUT));
    digitalWrite(BR_DIG, BR_OUT > 0 ? HIGH : LOW);
}
