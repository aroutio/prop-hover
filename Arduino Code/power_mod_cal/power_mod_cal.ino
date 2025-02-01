const int analogPin = A1; // Analog pin to read voltage

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) { // Check for command from Python
    char command = Serial.read(); // Read the command
    if (command == 'R') {         // 'R' indicates "Read voltage"
      int voltage = analogRead(analogPin); // Read the analog value
      Serial.println(String(voltage));    // Send the voltage back to Python
    }
  }
}
