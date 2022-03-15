#define DRIBBLER_PIN 18
#define DRIBBLER_WAIT 3000

void setup() {
  // put your setup code here, to run once:
  pinMode(DRIBBLER_PIN, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(DRIBBLER_PIN, 1000);
  analogWrite(DRIBBLER_PIN, 32);
  delay(3000);
  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for (int i = 32; i < 64; i++){
//    analogWrite(DRIBBLER_PIN, i);
//    delay(1);
//  }
  analogWrite(DRIBBLER_PIN, 64);
  delay(2000);
    analogWrite(DRIBBLER_PIN, 32);
//  for (int i = 64; i > 32; i--){
//    analogWrite(DRIBBLER_PIN, i);
//    delay(1);
//  }
  delay(2000);
  
//  for (int i = 8192; i < 16384; i++){
//    analogWrite(DRIBBLER_PIN, i);
//    delay(5);
//  }
//  for (int i = 16384; i < 8192; i--){
//    analogWrite(DRIBBLER_PIN, i);
//    delay(5);
//  }
//  
  
}
