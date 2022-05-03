#include "PID.h"

PID::PID(double p, double i, double d, double mult) {
  kp = p;
  ki = i;
  kd = d;
  integral = 0;
  integralMult = mult;
  lastTime = millis();
}

double PID::update(double error) {
    elapsedTime = (double)(millis() - lastTime) / 1000;

    proportional = error;
    integral = integral*integralMult + error;
    derivative = (error - lastError); // / elapsedTime;
    
    lastTime = millis();
    lastError = error;
    return kp * proportional + ki * integral + kd * derivative;
}

void PID::resetIntegral() {
  integral = 0;
}