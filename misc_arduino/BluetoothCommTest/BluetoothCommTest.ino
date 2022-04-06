int led = LED_BUILTIN;
char c;
#define EN_PIN 6
#define KICK_PIN 12



// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  Serial.println("ready");
  Serial2.begin(115200);
  pinMode(KICK_PIN, OUTPUT);
  digitalWrite(KICK_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

}

// the loop function runs over and over again forever
void loop() {
   Serial2.write(255);
   while (Serial2.available()){
      Serial.println(Serial2.read());
   }
   
      
     
}
