int led = LED_BUILTIN;
char c;
#define EN_PIN 6



// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  Serial.println("ready");
  Serial2.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

}

// the loop function runs over and over again forever
void loop() {
   while (Serial.available()) {
      Serial2.write(Serial.read());
   }
   while (Serial2.available()){
      Serial.write(Serial2.read());
   }
      
     
}
