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
    // Serial.print("P: ");
    // Serial.print(kp * proportional);
    // Serial.print(" I: ");
    // Serial.print(ki * integral);
    // Serial.print(" D: ");
    // Serial.print(kd * derivative);
    // Serial.print(" Correction");
    // Serial.print(kp * proportional + ki * integral + kd * derivative);
    //     Serial.println();

    return kp * proportional + ki * integral + kd * derivative;
}

void PID::resetIntegral() {
  integral = 0;
}

void PID::setVals(double p, double i, double d){
  kp = p;
  ki = i;
  kd = d;
}