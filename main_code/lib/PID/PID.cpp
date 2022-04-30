#include "PID.h"

PID::PID(double p, double i, double d, double limit) {
  kp = p;
  ki = i;
  kd = d;
  integral = 0;
  integralLimit = limit;
  lastTime = millis();
}

double PID::update(double error) {
    elapsedTime = (double)(millis() - lastTime) / 1000;

    proportional = error;
    integral = 0.95*integral +  error;
    if (abs(integral) > abs(integralLimit)) {
        integral = integralLimit;
    }
    if ((error - lastError) != 0){
        derivative = (error - lastError); // / elapsedTime;
    } else {
        derivative = 0;
    }
    lastTime = millis();
    lastError = error;
    return kp * proportional + ki * integral + kd * derivative;
}

void PID::resetIntegral() {
  integral = 0;
}