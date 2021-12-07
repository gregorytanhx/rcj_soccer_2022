#include "TOF.h"

TOF::TOF(TwoWire &i2cPort, int shutdownPin, int interruptPin){
  sensor = SFEVL53L1X(i2cPort, shutdownPin, interruptPin);
  shutdownPin = shutdownPin;
  interruptPin = interruptPin;
  pinMode(shutdownPin, OUTPUT);
  pinMode(interruptPin, OUTPUT);
  digitalWrite(shutdownPin, LOW);
}

void TOF::init(int i2cAddress){
  digitalWrite(shutdownPin, HIGH);
  sensor.init();
  sensor.setI2CAddress(i2cAddress);
  sensor.setDistanceModeLong();
  sensor.setTimingBudgetInMs(TIME_BUDGET);
  sensor.setIntermeasurementPeriod(IMP);
}

int16_t TOF::read(){  
  sensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!sensor.checkForDataReady()) delay(1);
  int16_t distance = sensor.getDistance(); //Get the result of the measurement from the sensor
  sensor.clearInterrupt();
  sensor.stopRanging();

  return distance;
}

TOF_Array::TOF_Array(TwoWire &i2cPort){
  FrontTOF = TOF(i2cPort, SHUT_1, INT_1);
  BackTOF = TOF(i2cPort, SHUT_2, INT_2);
  LeftTOF = TOF(i2cPort, SHUT_3, INT_3);
  RightTOF = TOF(i2cPort, SHUT_4, INT_4);
}

void TOF_Array::init(){
  FrontTOF.init();
  delay(10);
  BackTOF.init();
  delay(10);
  LeftTOF.init();
  delay(10);
  RightTOF.init();
  delay(10);
}

void TOF_Array::update(){
  distances[0] = FrontTOF.read();
  distances[1] = RightTOF.read();
  distances[2] = BackTOF.read();
  distances[4] = LeftTOF.read();
}


