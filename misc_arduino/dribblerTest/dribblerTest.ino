#define DRIBBLER_PIN 1
#define DRIBBLER_WAIT 3000

void setup() {
  // put your setup code here, to run once:
  analogWriteResolution(8);
  pinMode(DRIBBLER_PIN, OUTPUT);
  
  analogWriteFrequency(DRIBBLER_PIN, 1000);
  analogWrite(DRIBBLER_PIN, 32);
  delay(4000);
  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for (int i = 32; i < 64; i++){
//    analogWrite(DRIBBLER_PIN, i);
//    delay(1);
//  }
//  analogWrite(DRIBBLER_PIN, 64);
//  
analogWrite(DRIBBLER_PIN, 64);

 
  
  
}
