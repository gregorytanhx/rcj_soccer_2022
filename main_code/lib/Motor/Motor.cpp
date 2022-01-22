#include "Motor.h"

Motor::Motor(int pwm, int dig) {
  pwmPin = pwm;
  digPin = dig;
}

void Motor::init() {
  pinMode(pwmPin, OUTPUT);
  pinMode(digPin, OUTPUT);
}

void Motor::update(int pwm) {
  pwmOut = pwm;
}

void Motor::move() {
  analogWrite(pwmPin, abs(pwmOut));
  digitalWriteFast(digPin, (pwmOut > 0 ? HIGH : LOW));
}

Motors::Motors() {
  FL_Motor = Motor(FL_DIG, FL_PWM);
  FR_Motor = Motor(FR_DIG, FR_PWM);
  BL_Motor = Motor(BL_DIG, BL_PWM);
  BR_Motor = Motor(BR_DIG, BR_PWM);
}

void Motors::setMove(float speed, float angle, float rotation) {
  float a = sinf(deg2rad(50 + angle)) * (1 / sinf(deg2rad(80)));
  float b = sinf(deg2rad(50 - angle)) * (1 / sinf(deg2rad(80)));

  float fl = a - rotation;
  float fr = b + rotation;
  float bl = b - rotation;
  float br = a + rotation;

  float factor = speed / max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
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
