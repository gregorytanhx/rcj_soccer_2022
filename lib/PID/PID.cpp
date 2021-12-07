#include "PID.h"

PID::PID(float kp, float ki, float kd){
  kp = kp;
  ki = ki;
  kd = kd;
  lastTime = millis()
}

void PID::update(float error){
  elapsedTime = (millis() - lastTime) / 1000;

  float proportional = error;
  integral += error * elapsedTime;
  float derivative = (error - lastError) / elapsedTime
  lastTime = millis();
  lastError = error;

  return kp * proportional + ki * integral + kd * derivative;

}