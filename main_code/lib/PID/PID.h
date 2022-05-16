#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Common.h>

class PID {
   public:
    PID(){};
    PID(double p, double i, double d, double mult = 1);
    double update(double error);
    void resetIntegral();
    void setVals(double p, double i, double d);

    double kp;
    double ki;
    double kd;

   private:
    int lastTime;
    double elapsedTime;
    double lastError = 0;
    double proportional;
    double integral;
    double derivative;
    double integralMult;
};

#endif