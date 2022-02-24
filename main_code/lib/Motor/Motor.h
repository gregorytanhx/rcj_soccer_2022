#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Common.h>
#include <Pins.h>

// struct for movement data to be sent from teensy to layer1
typedef struct MoveData {
    floatData speed;
    floatData angle;
    floatData rotation;
    MoveData(int moveSpeed, int moveAngle, int moveRotation) {
        speed.val = moveSpeed;
        angle.val = moveAngle;
        rotation.val = moveRotation;
    }
} MoveData;

// class to control single motor
class Motor {
   public:
    Motor() {}
    Motor(int pwm, int dig);
    void update(int pwm);
    void move();
    void init();

   private:
    int pwmPin;
    int digPin;
    int pwmOut = 0;
};

// class to control all motors
class Motors {
   public:
    Motors();
    Motor FL_Motor;
    Motor FR_Motor;
    Motor BL_Motor;
    Motor BR_Motor;
    void setMove(float speed, float angle, float angVel);
    void moveOut();
};

#endif