#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Common.h>
#include <IMU.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <utility/imumaths.h>
    

#define sendSerial Serial1
TwoWire Wire1(PB11, PB10);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);
void setup() {
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
    sendSerial.begin(9600);


    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections
         */
        sendSerial.print(
            "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);
    bno.setExtCrystalUse(true);
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);
    float heading = event.orientation.x;
    sendSerial.println(heading);
}