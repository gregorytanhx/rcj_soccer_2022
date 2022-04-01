#include "Motor.h"

void Motors::init(uint8_t id) {
    robotID = id;
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
                     float angSpeed) {
   
    // angle between motors is 100 from left to right, 80 from top to bottom
    if (angSpeed == -1.0) angSpeed = speed;

    a = sin(deg2rad(50 + angle)) * MOTOR_MULT;
    b = sin(deg2rad(50 - angle)) * MOTOR_MULT;

    fl = a * speed - angSpeed * rotation * 0.1;
    fr = b * speed + angSpeed * rotation * 0.1;
    bl = b * speed - angSpeed * rotation * 0.1;
    br = a * speed + angSpeed * rotation * 0.1;

    FL_OUT = round(fl);
    FR_OUT = round(fr);
    BL_OUT = round(bl);
    BR_OUT = round(br);
}

void Motors::moveOut() {
    
    if (robotID == 0) {
        // bot 1
        analogWrite(FL_PWM, abs(FL_OUT));
        digitalWrite(FL_DIG, FL_OUT > 0 ? HIGH : LOW);

        analogWrite(FR_PWM, abs(FR_OUT));
        digitalWrite(FR_DIG, FR_OUT > 0 ? HIGH : LOW);

        analogWrite(BL_PWM, abs(BL_OUT));
        digitalWrite(BL_DIG, BL_OUT > 0 ? HIGH : LOW);

        analogWrite(BR_PWM, abs(BR_OUT));
        digitalWrite(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
    } else {
        // bot 2
        analogWrite(FL_PWM, abs(FL_OUT));
        digitalWrite(FL_DIG, FL_OUT > 0 ? LOW : HIGH);

        analogWrite(FR_PWM, abs(FR_OUT));
        digitalWrite(FR_DIG, FR_OUT > 0 ? LOW : HIGH);

        analogWrite(BL_PWM, abs(BL_OUT));
        digitalWrite(BL_DIG, BL_OUT > 0 ? HIGH : LOW);

        analogWrite(BR_PWM, abs(BR_OUT));
        digitalWrite(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
    }
   
}
