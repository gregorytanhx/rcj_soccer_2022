#include "Timer.h"

Timer::Timer(long duration) {
  timerDuration = duration;
  lastTime = millis();
}

void Timer::update() {
  lastTime = millis();
}

bool Timer::timeHasPassed(bool Update = true) {
  if (millis() - timerDuration > lastTime) {
    if (Update) update();
    return true;
  }   
  return false;
}