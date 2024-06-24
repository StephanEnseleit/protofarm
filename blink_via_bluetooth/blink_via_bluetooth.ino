char command;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();
    // Perform actions based on received command
    if (command == 'A') {
       digitalWrite(LED_BUILTIN, HIGH);
    } else if (command == 'B') {
      digitalWrite(LED_BUILTIN, LOW);
    }
    // Add more commands as needed
  }
}