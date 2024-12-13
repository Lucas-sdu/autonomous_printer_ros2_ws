void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
  while (!Serial) {
    ; // Wait for the serial port to connect
  }
}

void loop() {
  int potValue = analogRead(A0);  // Read the value from the potentiometer
  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);  // Send the potentiometer value over serial
  
  delay(500);  // Delay for 500 milliseconds
}

