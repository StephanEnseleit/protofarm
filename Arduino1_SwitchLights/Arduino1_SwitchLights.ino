#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins

const byte address[6] = "00001";
char command; //command from code

void setup() {
  Serial.begin(9600); //initialization for the serial monitor
  
  radio.begin(); // initialization of the connection from the radio module
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
}

void loop() {
  bool buttonPressed;
  
  if (Serial.available() > 0) {
    command = Serial.read();
    Serial.println(command);
  }

  switch (command){
    case 'h':
      radio.read(&buttonPressed, sizeof(buttonPressed));
      Serial.println(buttonPressed);
      break;
    case 'e':
      Serial.println("exit homing");
      break;
  }
  
  /*if (radio.available()) {
    radio.read(&buttonPressed, sizeof(buttonPressed));
    Serial.println(buttonPressed);
  }*/
}