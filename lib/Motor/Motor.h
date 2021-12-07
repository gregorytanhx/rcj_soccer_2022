#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Pins.h>


typedef struct moveData {
  float speed = 0;
  float angle = 0;
  float rotation = 0;
} moveData;

class Motor{
  public: 
    Motor(int dig, int pwm);
    void update();
    void move();

  private:
    int pwmPin;
    int digPin;
    int pwmOut = 0;
}

class Motors{
  public:
    Motors();
    Motor FL_Motor;
    Motor FR_Motor;
    Motor BL_Motor;
    Motor BR_Motor;
    void setMove(float speed, float angle, float angVel);
    void angleCorrect(float angle);
    void moveOut();
  private:
    float cmp_kp;
    
}


