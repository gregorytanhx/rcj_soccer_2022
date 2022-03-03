#ifndef MYTIMER_H
#define MYTIMER_H

#include <Arduino.h>
#include <Common.h>

class MyTimer {
  public:
    MyTimer(long duration);
    bool timeHasPassed(bool Update = false);
    long timerDuration = 0;
    void update();
    long lastTime = 0;
};

#endif