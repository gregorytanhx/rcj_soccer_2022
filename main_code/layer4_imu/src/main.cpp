#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Common.h>
#include <IMU.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <utility/imumaths.h>


// HardwareSerial ser(PA10, PA9);
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);
void setup() {
    Serial.begin(9600);
    // if (!bno.begin()) {
    //     /* There was a problem detecting the BNO055 ... check your connections
    //      */
    //     sendSerial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //     while (1);
    // }

    // delay(1000);
    // bno.setExtCrystalUse(true);
}

void loop() {
    Serial.println("FUCK");
}