void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {
  if(Serial.available()){
    String data = Serial.readStringUntil(',');
    Serial.println("Received: " + data); // Display the received character in the Serial Monitor
  }
}
