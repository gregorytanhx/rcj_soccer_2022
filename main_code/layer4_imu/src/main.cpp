#include <Arduino.h>
#include <Common.h>
#include <Pins.h>
#include <IMU.h>
#include <Wire.h>

TwoWire Wire1(PB11, PB10);

IMU cmp(&Wire1);

#define Serial L4DebugSerial

CmpVal cmpVal;

void sendData() {
    cmpVal.val = (int) (cmp.readQuat() * 100);
    Serial2.write(IMU_SYNC_BYTE);
    Serial2.write(cmpVal.b, 2);
}

void i2cScanner() {
     byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 
}

void setup() {    
    Serial2.begin(STM32_BAUD);
#ifdef DEBUG
    Serial.begin(9600);
#endif
    Serial.println("test");
    delay(3000);
    cmp.begin();
    pinMode(STM32_LED, OUTPUT);
    digitalWrite(STM32_LED, HIGH);
    delay(1000);
     digitalWrite(STM32_LED, LOW);
}
long lastPrintTime = 0;
void loop() {
   sendData();
    Serial.println(cmp.readQuat());
  
}



