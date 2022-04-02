#include "Motor.h"

void Motors::init(uint8_t id) {
    robotID = id;
    // // use 12 bit resolution for motor speed
    // analogWriteResolution(12);
    // analogWriteFrequency(14648.437);

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
        digitalWriteFast(FL_DIG, FL_OUT > 0 ? HIGH : LOW);

        analogWrite(FR_PWM, abs(FR_OUT));
        digitalWriteFast(FR_DIG, FR_OUT > 0 ? HIGH : LOW);

        analogWrite(BL_PWM, abs(BL_OUT));
        digitalWriteFast(BL_DIG, BL_OUT > 0 ? HIGH : LOW);

        analogWrite(BR_PWM, abs(BR_OUT));
        digitalWriteFast(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
    } else {
        // bot 2
        analogWrite(FL_PWM, abs(FL_OUT));
        digitalWriteFast(FL_DIG, FL_OUT > 0 ? LOW : HIGH);

        analogWrite(FR_PWM, abs(FR_OUT));
        digitalWriteFast(FR_DIG, FR_OUT > 0 ? LOW : HIGH);

        analogWrite(BL_PWM, abs(BL_OUT));
        digitalWriteFast(BL_DIG, BL_OUT > 0 ? HIGH : LOW);

        analogWrite(BR_PWM, abs(BR_OUT));
        digitalWriteFast(BR_DIG, BR_OUT > 0 ? LOW : HIGH);
    }
   
}
