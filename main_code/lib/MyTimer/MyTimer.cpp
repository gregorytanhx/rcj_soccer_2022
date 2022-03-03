#include "MyTimer.h"

MyTimer::MyTimer(long duration) {
  timerDuration = duration;
  lastTime = millis();
}

void MyTimer::update() {
  lastTime = millis();
}

bool MyTimer::timeHasPassed(bool Update = true) {
  if (millis() - timerDuration > lastTime) {
    if (Update) update();
    return true;
  }   
  return false;
}