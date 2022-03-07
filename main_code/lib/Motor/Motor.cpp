#include "Motor.h"

Motor::Motor(int pwm, int dig, int pol) {
    pwmPin = pwm;
    digPin = dig;
    polarity = pol;

}

void Motor::init() {
    pinMode(pwmPin, OUTPUT);
    pinMode(digPin, OUTPUT);
}

void Motor::update(int pwm) {
    pwmOut = pwm * polarity;
}

void Motor::move() {
    analogWrite(pwmPin, abs(pwmOut));
    digitalWrite(digPin, (pwmOut > 0 ? HIGH : LOW));
}

Motors::Motors() {
    FL_Motor = Motor(FL_DIG, FL_PWM, 1);
    FR_Motor = Motor(FR_DIG, FR_PWM, -1);
    BL_Motor = Motor(BL_DIG, BL_PWM, 1);
    BR_Motor = Motor(BR_DIG, BR_PWM, -1);
}

void Motors::init() {
    FL_Motor.init();
    FR_Motor.init();
    BL_Motor.init();
    BR_Motor.init();
}

void Motors::setMove(float speed, float angle, float rotation) {
    float outSpeed = map(speed, 0, 100, 0, MAX_PWM);
    
    float a = sin(deg2rad(50 + angle)) * (1 / sin(deg2rad(80)));
    float b = sin(deg2rad(50 - angle)) * (1 / sin(deg2rad(80)));

    float fl = a - rotation;
    float fr = b + rotation;
    float bl = b - rotation;
    float br = a + rotation;

    float factor = outSpeed / max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
    FL_Motor.update(round(fl * factor));
    FR_Motor.update(round(fr * factor));
    BL_Motor.update(round(bl * factor));
    BR_Motor.update(round(br * factor));
}

void Motors::moveOut() {
    FL_Motor.move();
    FR_Motor.move();
    BL_Motor.move();
    BR_Motor.move();
}
