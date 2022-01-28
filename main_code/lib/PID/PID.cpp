#include "PID.h"

PID::PID(float p, float i, float d, float limit = 0) {
  kp = p;
  ki = i;
  kd = d;
  integral = 0;
  integralLimit = limit
  lastTime = millis();
}

float PID::update(float error) {
  elapsedTime = (millis() - lastTime) / 1000;

  float proportional = error;
  integral += error * elapsedTime;
  if (abs(integral) > abs(integralLimit)) {
      integral = integralLimit;
  }
  float derivative = (error - lastError) / elapsedTime;
  lastTime = millis();
  lastError = error;

  return kp * proportional + ki * integral + kd * derivative;

}

void PID::resetIntegral() {
  integral = 0;
}