#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Pins.h>
#include <TOF.h>
#include <ComponentObject.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>


TwoWire i2cBus(LAYER4_SDA, LAYER4_SCL);

TOF_Array tofArray(i2cBus);

void i2c_scanner() {
    byte error, address;
    int nDevices;

    L4DebugSerial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        i2cBus.beginTransmission(address);
        error = i2cBus.endTransmission();

        if (error == 0) {
            L4DebugSerial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            L4DebugSerial.print(address, HEX);
            L4DebugSerial.println("  !");

            nDevices++;
        } else if (error == 4) {
            L4DebugSerial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            L4DebugSerial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        L4DebugSerial.println("No I2C devices found");
    else
        L4DebugSerial.println("done");
}

void setup() {
    L4CommSerial.begin(STM32_BAUD);
#ifdef DEBUG
    L4DebugSerial.begin(9600);
#endif
    i2cBus.begin();
    i2cBus.setClock(400000);   
    tofArray.init();
}


void loop() {
    tofArray.update();
    tofArray.print();

}