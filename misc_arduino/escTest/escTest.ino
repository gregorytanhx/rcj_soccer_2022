static const int OUT_PIN = 23;

const float LOWER_LIMIT = 8192.0;
const float UPPER_LIMIT = 16384.0;

void setup() {
  // put your setup code here, to run once:
  //analogWriteResolution(16);
  pinMode(OUT_PIN, OUTPUT);
  analogWriteFrequency(OUT_PIN, 1000);
  analogWrite(OUT_PIN, 32);
  delay(3000);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float T = 1000000.0/1000.0;
  analogWrite(OUT_PIN, 64);
  
//  for (float value = LOWER_LIMIT; value < UPPER_LIMIT; value++) {
//    analogWrite(OUT_PIN, value);
//    Serial.println(value);
//    delay(5);
//  }
//
//  for (float value = UPPER_LIMIT; value > LOWER_LIMIT; value--) {
//    analogWrite(OUT_PIN, value);
//    Serial.println(value);
//    delay(5);
//  }
  

}
