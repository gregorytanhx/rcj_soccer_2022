int led = LED_BUILTIN;
char c;

typedef union dataBuffer {
  uint16_t vals[2];
  byte bytes[sizeof(vals)];
} dataBuffer;

dataBuffer buffer;


// the setup function runs once when you press reset or power the board
void setup() {
  // angle
  buffer.vals[0] = 0;
  // distance
 
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  Serial.println("ready");
  Serial2.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

// the loop function runs over and over again forever
void loop() {
  buffer.vals[1] = 500;
  if (buffer.vals[0] < 360) {
    buffer.vals[0]++;
  } else {
    buffer.vals[0] = 0;
  }
  Serial.println();
  Serial2.write(buffer.bytes, sizeof(buffer.bytes));
  Serial2.write('\n');
  Serial.flush();
 
  

 
//  while (Serial1.available() > 0){
//    Serial.println(Serial1.read());
//  }
                        // wait for a second
}
