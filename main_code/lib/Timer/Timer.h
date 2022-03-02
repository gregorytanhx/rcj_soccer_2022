#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <Common.h>

class Timer {
  public:
    Timer(long duration);
    bool timeHasPassed(bool Update = true);
    long timerDuration = 0;
    void update();
    long lastTime = 0;
};

#endif