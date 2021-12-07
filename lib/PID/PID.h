#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Common.h>

class PID{
  public:
    PID(float kp, float ki, float kd)
    float update(error)
  
    float kp;
    float ki;
    float kd;

  private:
    int lastTime;
    int elapsedTime;
    float lastError = 0;
    float integral = 0;
}