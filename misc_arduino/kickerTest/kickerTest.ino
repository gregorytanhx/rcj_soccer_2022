#define KICKER_PIN 22

int lightVal;
void setup() {
  // put your setup code here, to run once:
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
//  lightVal = analogRead(A9);
//  Serial.println(lightVal);
//  if (lightVal < 20) {
//    digitalWrite(KICKER_PIN, HIGH);
//    delay(500);
//  }
  digitalWrite(KICKER_PIN, HIGH);
  delay(1000);
  digitalWrite(KICKER_PIN, LOW);
  delay(1000);

}
