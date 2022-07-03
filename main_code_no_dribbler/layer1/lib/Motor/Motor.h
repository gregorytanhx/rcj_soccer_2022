#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Common.h>
#include <Pins.h>


// class to control all motors
class Motors {
   public:
    void setMove(float speed, float angle, float angVel, float angSpeed = -1.0);
    void moveOut();
    void begin(uint8_t id);
    uint8_t robotID;
    float a, b, fl, fr, bl, br;
    float FL_OUT, FR_OUT, BR_OUT, BL_OUT;
};

#endif