#define KICKER_PIN 12

int lightVal;
long lastKickTime = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  lightVal = analogRead(A9);
  Serial.println(lightVal);
  if (lightVal < 20) {
    digitalWrite(KICKER_PIN, LOW);
    lastKickTime = millis();
  }
  if (millis() - lastKickTime > 200){
    digitalWrite(KICKER_PIN, HIGH);
  }
  

}
